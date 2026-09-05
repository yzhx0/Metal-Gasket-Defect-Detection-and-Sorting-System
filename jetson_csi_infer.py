"""Jetson Nano TensorRT inspection service for metal washers.

The STM32 owns line timing and sends a sequenced trigger.  This service uses
the latest camera frame, classifies notch/deformation defects and returns a
sequenced result.  Duplicate triggers are answered from a short-lived cache,
so a lost UART response never causes the same part to be inferred twice.
"""

import json
import os
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # 初始化 CUDA 上下文
import serial
import time
import threading
import queue
from typing import List, Tuple

from inspection_protocol import (
    MSG_ACK,
    MSG_RESULT,
    MSG_TRIGGER,
    RESULT_DEFORMATION,
    RESULT_NORMAL,
    RESULT_NOTCH,
    RESULT_VISION_ERROR,
    ResultCache,
    StreamParser,
    build_frame,
)

# ── 配置 ──────────────────────────────────────────────────────────────────────
ENGINE_PATH = os.getenv("INSPECTION_ENGINE", "washer_yolov8n_fp16.engine")
INPUT_W, INPUT_H = 640, 640
CONF_THRESH = float(os.getenv("INSPECTION_CONFIDENCE", "0.45"))
IOU_THRESH = float(os.getenv("INSPECTION_IOU", "0.45"))
CAM_INDEX = int(os.getenv("INSPECTION_CAMERA", "0"))
CAM_W, CAM_H = 640, 480  # 摄像头分辨率 (模型输入 640x640，无需 1080p)

# ── 串口配置 ───────────────────────────────────────────────────────────────
SERIAL_PORT = os.getenv("INSPECTION_SERIAL", "/dev/ttyTHS1")
SERIAL_BAUD = int(os.getenv("INSPECTION_BAUD", "115200"))
SERIAL_TIMEOUT = 0.02

# Custom model convention: class 0=notch, class 1=deformation.
CLASS_NAMES = os.getenv(
    "INSPECTION_CLASS_NAMES", "notch,deformation"
).split(",")
DEFECT_CLASS_IDS = set(
    int(value)
    for value in os.getenv("INSPECTION_DEFECT_CLASS_IDS", "0,1").split(",")
    if value.strip()
)

# ── 调试开关 ───────────────────────────────────────────────────────────────
SHOW_WINDOW = os.getenv("INSPECTION_SHOW_WINDOW", "0") == "1"
DEBUG_RAW_HEX = os.getenv("INSPECTION_DEBUG_UART", "0") == "1"
# ── 心跳 / 链路监控 ─────────────────────────────────────────────────────────
HEARTBEAT_PERIOD_S = float(os.getenv("INSPECTION_HEARTBEAT_PERIOD_S", "1.0"))
LINK_SILENT_TIMEOUT_S = float(
    os.getenv("INSPECTION_LINK_SILENT_TIMEOUT_S", "5.0")
)

# ── 缺陷图像留存 ───────────────────────────────────────────────────────────
SAVE_IMAGES = os.getenv("INSPECTION_SAVE_IMAGES", "1") == "1"
SAVE_ERROR_IMAGES = os.getenv("INSPECTION_SAVE_ERROR_IMAGES", "1") == "1"
IMAGE_DIR = os.getenv("INSPECTION_IMAGE_DIR", "defect_images")
IMAGE_QUEUE_SIZE = 32
JPEG_QUALITY = 90

def _hex_str(data: bytes) -> str:
    """bytes → 空格分隔的大写16进制字符串 (兼容 Python 3.6)"""
    return " ".join(f"{b:02X}" for b in data)


# ── TensorRT 引擎加载 ───────────────────────────────────────────────────────
class TRTEngine:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        # 分配主机/设备缓冲区
        self.inputs, self.outputs, self.bindings, self.stream = (
            [],
            [],
            [],
            cuda.Stream(),
        )
        self.output_shape = None  # 缓存第一个输出的 shape

        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            self.bindings.append(int(device_mem))
            if self.engine.binding_is_input(binding):
                self.inputs.append({"host": host_mem, "device": device_mem})
            else:
                self.outputs.append({"host": host_mem, "device": device_mem})
                if self.output_shape is None:
                    self.output_shape = self.engine.get_binding_shape(binding)

    def infer(self, img_chw: np.ndarray) -> List[np.ndarray]:
        np.copyto(self.inputs[0]["host"], img_chw.ravel())
        for inp in self.inputs:
            cuda.memcpy_htod_async(inp["device"], inp["host"], self.stream)
        self.context.execute_async_v2(self.bindings, self.stream.handle, None)
        for out in self.outputs:
            cuda.memcpy_dtoh_async(out["host"], out["device"], self.stream)
        self.stream.synchronize()
        return [out["host"] for out in self.outputs]


# ── 预处理 ────────────────────────────────────────────────────────────────────
def preprocess(frame: np.ndarray) -> Tuple[np.ndarray, float, tuple]:
    """letterbox 缩放 → CHW float32，返回 (blob, scale, pad)"""
    h0, w0 = frame.shape[:2]
    scale = min(INPUT_H / h0, INPUT_W / w0)
    nh, nw = int(round(h0 * scale)), int(round(w0 * scale))
    img = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)

    # 填充到 640x640
    dh, dw = (INPUT_H - nh) // 2, (INPUT_W - nw) // 2
    img = cv2.copyMakeBorder(
        img,
        dh,
        INPUT_H - nh - dh,
        dw,
        INPUT_W - nw - dw,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = img.transpose(2, 0, 1)  # HWC → CHW
    img = np.ascontiguousarray(img[None])  # 加 batch 维
    return img, scale, (dw, dh)


# ── 后处理（YOLOv5 / YOLOv8 输出格式）────────────────────────────────────────
def postprocess(raw: np.ndarray, scale: float, pad: tuple, orig_h: int, orig_w: int):
    """
    raw shape: (1, 25200, 85) for YOLOv5
               (1, 84, 8400)  for YOLOv8  → 自动转置
    返回 list of (x1,y1,x2,y2,conf,cls_id)
    """
    pred = raw.squeeze()  # 去掉 batch 维

    # YOLOv8: [84, 8400] → [8400, 84]
    if pred.shape[0] < pred.shape[1]:
        pred = pred.T
        # YOLOv8 无 objectness，直接取类别最大值
        boxes = pred[:, :4]
        scores = pred[:, 4:]
        cls_ids = scores.argmax(axis=1)
        confs = scores.max(axis=1)
    else:
        # YOLOv5: [25200, 85]，第4列是 objectness
        obj_conf = pred[:, 4]
        cls_conf = pred[:, 5:].max(axis=1)
        confs = obj_conf * cls_conf
        cls_ids = pred[:, 5:].argmax(axis=1)
        boxes = pred[:, :4]

    mask = confs > CONF_THRESH
    boxes, confs, cls_ids = boxes[mask], confs[mask], cls_ids[mask]

    if len(boxes) == 0:
        return []

    # xywh → xyxy（中心格式）
    dw, dh = pad
    x1 = (boxes[:, 0] - boxes[:, 2] / 2 - dw) / scale
    y1 = (boxes[:, 1] - boxes[:, 3] / 2 - dh) / scale
    x2 = (boxes[:, 0] + boxes[:, 2] / 2 - dw) / scale
    y2 = (boxes[:, 1] + boxes[:, 3] / 2 - dh) / scale
    x1, y1 = np.clip(x1, 0, orig_w), np.clip(y1, 0, orig_h)
    x2, y2 = np.clip(x2, 0, orig_w), np.clip(y2, 0, orig_h)

    rects = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)
    # OpenCV NMSBoxes expects [x, y, width, height], not xyxy.
    nms_rects = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1).astype(np.float32)
    keep = cv2.dnn.NMSBoxes(
        nms_rects.tolist(), confs.tolist(), CONF_THRESH, IOU_THRESH
    )
    if len(keep) == 0:
        return []
    keep = keep.flatten()
    return [(rects[i], confs[i], cls_ids[i]) for i in keep]


# ── 可视化 ────────────────────────────────────────────────────────────────────
def draw(frame, detections):
    for box, conf, cls_id in detections:
        x1, y1, x2, y2 = map(int, box)
        class_index = int(cls_id)
        class_name = (
            CLASS_NAMES[class_index]
            if class_index < len(CLASS_NAMES)
            else "class_%d" % class_index
        )
        label = f"{class_name} {conf:.2f}"
        color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
        cv2.putText(
            frame, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1
        )
    return frame


class LatestFrameCamera(object):
    """Continuously drains the UVC buffer and exposes only the newest frame.

    A run of failed reads (e.g. after a USB cable pull or a driver glitch)
    triggers a capture reopen so the line recovers without a service restart.
    """

    REOPEN_AFTER_FAILURES = 60

    def __init__(self, camera_index, metrics=None):
        self._index = camera_index
        self._metrics = metrics if metrics is not None else {}
        self._reopen_count = 0
        self.capture = cv2.VideoCapture(camera_index)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        self._lock = threading.Lock()
        self._frame = None
        self._timestamp = 0.0
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if not self.capture.isOpened():
            raise RuntimeError("cannot open /dev/video%d" % self._index)
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
        self._thread.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with self._lock:
                if self._frame is not None:
                    return
            time.sleep(0.01)
        raise RuntimeError("camera produced no frames during startup")

    def _run(self):
        consecutive_failures = 0
        while not self._stop.is_set():
            ok, frame = self.capture.read()
            if not ok or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= self.REOPEN_AFTER_FAILURES:
                    self._reopen()
                    consecutive_failures = 0
                else:
                    time.sleep(0.01)
                continue
            consecutive_failures = 0
            with self._lock:
                self._frame = frame
                self._timestamp = time.monotonic()

    def _reopen(self):
        self._reopen_count += 1
        self._metrics["camera_reopens"] = self._reopen_count
        emit_event("camera_reopen", attempt=self._reopen_count)
        self.capture.release()
        time.sleep(0.2)
        capture = cv2.VideoCapture(self._index)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        opened = capture.isOpened()
        with self._lock:
            self.capture = capture
            self._frame = None
            self._timestamp = 0.0
        emit_event("camera_reopened", attempt=self._reopen_count, ok=opened)

    def latest(self, max_age_s=0.20):
        with self._lock:
            if self._frame is None:
                return None, None
            if time.monotonic() - self._timestamp > max_age_s:
                return None, self._timestamp
            return self._frame.copy(), self._timestamp

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.capture.release()


def emit_event(event, **fields):
    record = {"event": event, "timestamp": time.time()}
    record.update(fields)
    print(json.dumps(record, ensure_ascii=False, sort_keys=True), flush=True)


def serial_reader(ser, frame_queue, stop_event, metrics):
    parser = StreamParser()
    while not stop_event.is_set():
        try:
            chunk = ser.read(max(1, min(ser.in_waiting, 64)))
            if not chunk:
                continue
            if DEBUG_RAW_HEX:
                emit_event("uart_rx", data=_hex_str(chunk))
            for frame in parser.feed(chunk):
                metrics["last_frame_at"] = time.monotonic()
                try:
                    frame_queue.put_nowait((frame, time.monotonic()))
                except queue.Full:
                    metrics["queue_overflow"] += 1
                    emit_event("queue_overflow", sequence=frame.sequence)
            metrics["checksum_errors"] = parser.checksum_errors
            metrics["discarded_bytes"] = parser.discarded_bytes
        except serial.SerialException as exc:
            metrics["serial_errors"] += 1
            emit_event("serial_failure", error=str(exc))
            stop_event.set()
            break
    metrics["checksum_errors"] = parser.checksum_errors
    metrics["discarded_bytes"] = parser.discarded_bytes


def defect_image_name(sequence, code):
    """Timestamped, traceable file name for a defect frame."""
    ts = time.time()
    stamp = time.strftime("%Y%m%dT%H%M%S", time.localtime(ts))
    millis = int((ts * 1000.0) % 1000.0)
    return "%s.%03d_seq%03d_code%02X.jpg" % (stamp, millis, sequence, code)


def image_writer(image_queue, stop_event, metrics):
    """Persist defect frames off the inference latency path."""
    while not stop_event.is_set() or not image_queue.empty():
        try:
            item = image_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        frame, name = item
        try:
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if not ok:
                raise RuntimeError("jpeg encode failed")
            with open(os.path.join(IMAGE_DIR, name), "wb") as handle:
                handle.write(encoded.tobytes())
            metrics["saved_images"] += 1
        except Exception as exc:
            metrics["save_errors"] += 1
            emit_event("image_save_error", name=name, error=str(exc))


def heartbeat_sender(ser, stop_event, metrics, tx_lock):
    """Periodic MCU-facing heartbeat carrying RX link quality in AUX.

    Also supervises the reverse direction: a silent MCU (no frame parsed by
    the reader) raises a `stm32_silent` event once per silent period.
    """
    sequence = 0
    silent_reported = False
    emit_event("heartbeat_start", period_s=HEARTBEAT_PERIOD_S)
    while not stop_event.is_set():
        try:
            with tx_lock:
                ser.write(
                    build_frame(
                        MSG_HEARTBEAT,
                        sequence & 0xFF,
                        0,
                        metrics["checksum_errors"] & 0xFF,
                    )
                )
                ser.flush()
            sequence += 1
        except serial.SerialException as exc:
            metrics["serial_errors"] += 1
            emit_event("heartbeat_send_failure", error=str(exc))

        last_frame_at = metrics["last_frame_at"]
        silent = (
            last_frame_at > 0.0
            and (time.monotonic() - last_frame_at) > LINK_SILENT_TIMEOUT_S
        )
        if silent and not silent_reported:
            emit_event("stm32_silent", silent_for_s=LINK_SILENT_TIMEOUT_S)
            silent_reported = True
        elif not silent:
            silent_reported = False

        stop_event.wait(HEARTBEAT_PERIOD_S)


def classify(detections):
    defects = [
        detection
        for detection in detections
        if int(detection[2]) in DEFECT_CLASS_IDS
    ]
    if not defects:
        return RESULT_NORMAL, 0

    _, confidence, class_id = max(defects, key=lambda detection: detection[1])
    result = (
        RESULT_NOTCH if int(class_id) == 0 else RESULT_DEFORMATION
    )
    confidence_percent = min(100, max(0, int(round(float(confidence) * 100))))
    return result, confidence_percent


def main():
    emit_event(
        "startup",
        engine=ENGINE_PATH,
        serial=SERIAL_PORT,
        baud=SERIAL_BAUD,
        confidence=CONF_THRESH,
        heartbeat_period_s=HEARTBEAT_PERIOD_S,
        save_images=SAVE_IMAGES,
        image_dir=IMAGE_DIR,
    )
    engine = TRTEngine(ENGINE_PATH)
    # Warm-up moves one-off CUDA allocation outside the production latency path.
    engine.infer(np.zeros((1, 3, INPUT_H, INPUT_W), dtype=np.float32))

    frame_queue = queue.Queue(maxsize=32)
    image_queue = queue.Queue(maxsize=IMAGE_QUEUE_SIZE)
    stop_event = threading.Event()
    tx_lock = threading.Lock()
    metrics = {
        "inspections": 0,
        "duplicates": 0,
        "vision_errors": 0,
        "serial_errors": 0,
        "queue_overflow": 0,
        "saved_images": 0,
        "save_errors": 0,
        "image_drops": 0,
        "camera_reopens": 0,
        "checksum_errors": 0,
        "discarded_bytes": 0,
        "last_frame_at": 0.0,
    }
    if SAVE_IMAGES:
        os.makedirs(IMAGE_DIR, exist_ok=True)

    camera = LatestFrameCamera(CAM_INDEX, metrics)
    camera.start()
    ser = serial.Serial(
        SERIAL_PORT,
        SERIAL_BAUD,
        timeout=SERIAL_TIMEOUT,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        write_timeout=0.1,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    result_cache = ResultCache(capacity=32, ttl_s=5.0)
    reader_thread = threading.Thread(
        target=serial_reader,
        args=(ser, frame_queue, stop_event, metrics),
        name="uart-reader",
        daemon=True,
    )
    reader_thread.start()
    heartbeat_thread = threading.Thread(
        target=heartbeat_sender,
        args=(ser, stop_event, metrics, tx_lock),
        name="heartbeat",
        daemon=True,
    )
    heartbeat_thread.start()
    writer_thread = None
    if SAVE_IMAGES:
        writer_thread = threading.Thread(
            target=image_writer,
            args=(image_queue, stop_event, metrics),
            name="image-writer",
            daemon=True,
        )
        writer_thread.start()
    emit_event("ready")

    try:
        while not stop_event.is_set():
            try:
                message, received_at = frame_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            if message.message_type == MSG_ACK:
                result_cache.acknowledge(message.sequence)
                continue
            if message.message_type != MSG_TRIGGER:
                emit_event(
                    "unexpected_message",
                    message_type=message.message_type,
                    sequence=message.sequence,
                )
                continue

            cached = result_cache.get(message.sequence)
            if cached is not None:
                with tx_lock:
                    ser.write(cached)
                    ser.flush()
                metrics["duplicates"] += 1
                emit_event("duplicate_trigger", sequence=message.sequence)
                continue

            result_code = RESULT_VISION_ERROR
            confidence = 0
            detections = []
            frame = None
            try:
                frame, captured_at = camera.latest()
                if frame is None:
                    raise RuntimeError("latest camera frame is stale or unavailable")
                orig_h, orig_w = frame.shape[:2]
                blob, scale, pad = preprocess(frame)
                raw_outputs = engine.infer(blob)
                pred = raw_outputs[0].reshape(engine.output_shape)
                detections = postprocess(pred, scale, pad, orig_h, orig_w)
                result_code, confidence = classify(detections)
            except Exception as exc:
                metrics["vision_errors"] += 1
                emit_event(
                    "inspection_error",
                    sequence=message.sequence,
                    error=str(exc),
                )

            if (
                SAVE_IMAGES
                and frame is not None
                and result_code != RESULT_NORMAL
                and (result_code != RESULT_VISION_ERROR or SAVE_ERROR_IMAGES)
            ):
                name = defect_image_name(message.sequence, result_code)
                try:
                    image_queue.put_nowait((frame, name))
                except queue.Full:
                    metrics["image_drops"] += 1
                    emit_event(
                        "image_queue_overflow", sequence=message.sequence
                    )

            response = build_frame(
                MSG_RESULT, message.sequence, result_code, confidence
            )
            result_cache.put(message.sequence, response)
            with tx_lock:
                ser.write(response)
                ser.flush()
            metrics["inspections"] += 1
            latency_ms = (time.monotonic() - received_at) * 1000.0
            emit_event(
                "inspection_result",
                sequence=message.sequence,
                result=result_code,
                confidence=confidence,
                detections=len(detections),
                latency_ms=round(latency_ms, 2),
                over_budget=latency_ms > 195.0,
            )

            if SHOW_WINDOW and frame is not None:
                cv2.imshow("washer inspection", draw(frame, detections))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        emit_event("shutdown_requested")
    finally:
        stop_event.set()
        reader_thread.join(timeout=1.0)
        heartbeat_thread.join(timeout=1.0)
        if writer_thread is not None:
            writer_thread.join(timeout=2.0)
        camera.close()
        ser.close()
        cv2.destroyAllWindows()
        emit_event("shutdown", **metrics)


if __name__ == "__main__":
    main()
