#!/usr/bin/env python3
import ctypes
import json
import socket
import struct
import cv2
import numpy as np
import tensorrt as trt

ENGINE_PATH = "./yolov8n-pose.engine"
HOST = "127.0.0.1"
PORT = 5555
CONF_THRES = 0.5
IOU_THRES = 0.45

# Chargement direct du runtime CUDA système (aucun package externe requis)
cudart = ctypes.CDLL("libcudart.so")
cudart.cudaSetDevice.argtypes = [ctypes.c_int]
cudart.cudaSetDevice.restype = ctypes.c_int
cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
cudart.cudaMalloc.restype = ctypes.c_int
cudart.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
cudart.cudaMemcpy.restype = ctypes.c_int
cudart.cudaFree.argtypes = [ctypes.c_void_p]
cudart.cudaFree.restype = ctypes.c_int

CUDA_MEMCPY_HOST_TO_DEVICE = 1
CUDA_MEMCPY_DEVICE_TO_HOST = 2


class TrtYoloPose:
    def __init__(self, engine_path):
        # Initialisation du contexte CUDA sur le GPU 0
        assert cudart.cudaSetDevice(0) == 0, "Erreur cudaSetDevice(0)"
        cudart.cudaFree(None)

        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()

        self.bindings = []
        self.inputs = []
        self.outputs = []

        for i in range(self.engine.num_bindings):
            shape = tuple(self.engine.get_binding_shape(i))
            dtype = trt.nptype(self.engine.get_binding_dtype(i))
            size = int(np.prod(shape))
            host_mem = np.empty(size, dtype=dtype)

            dev_ptr = ctypes.c_void_p()
            err = cudart.cudaMalloc(ctypes.byref(dev_ptr), ctypes.c_size_t(host_mem.nbytes))
            assert err == 0, f"Erreur cudaMalloc : code {err}"

            self.bindings.append(dev_ptr.value)
            entry = {"host": host_mem, "device": dev_ptr, "shape": shape}
            if self.engine.binding_is_input(i):
                self.inputs.append(entry)
            else:
                self.outputs.append(entry)

        self.in_h = self.inputs[0]["shape"][2]
        self.in_w = self.inputs[0]["shape"][3]
        print(f"Moteur TensorRT chargé | Entrée: {self.inputs[0]['shape']} | Sortie: {self.outputs[0]['shape']}")

    def preprocess(self, img_bgr):
        h, w = img_bgr.shape[:2]
        scale = min(self.in_w / w, self.in_h / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)

        canvas = np.full((self.in_h, self.in_w, 3), 114, dtype=np.uint8)
        pad_x = (self.in_w - nw) // 2
        pad_y = (self.in_h - nh) // 2
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized

        blob = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
        blob = np.ascontiguousarray(blob, dtype=np.float32) / 255.0
        return blob, scale, pad_x, pad_y

    def infer(self, img_bgr):
        blob, scale, pad_x, pad_y = self.preprocess(img_bgr)
        inp = self.inputs[0]
        out = self.outputs[0]

        np.copyto(inp["host"], blob.ravel())

        cudart.cudaMemcpy(
            inp["device"],
            inp["host"].ctypes.data_as(ctypes.c_void_p),
            ctypes.c_size_t(inp["host"].nbytes),
            CUDA_MEMCPY_HOST_TO_DEVICE,
        )
        self.context.execute_v2(bindings=self.bindings)
        cudart.cudaMemcpy(
            out["host"].ctypes.data_as(ctypes.c_void_p),
            out["device"],
            ctypes.c_size_t(out["host"].nbytes),
            CUDA_MEMCPY_DEVICE_TO_HOST,
        )

        raw = out["host"].reshape(out["shape"])[0]
        return self.postprocess(raw, scale, pad_x, pad_y)

    def postprocess(self, raw, scale, pad_x, pad_y):
        detections = []

        # Cas 1 : Modèle YOLO26 end-to-end sans NMS -> shape (300, 57)
        if raw.shape[-1] == 57:
            for row in raw:
                score = float(row[4])
                if score < CONF_THRES:
                    continue
                x1 = (row[0] - pad_x) / scale
                y1 = (row[1] - pad_y) / scale
                x2 = (row[2] - pad_x) / scale
                y2 = (row[3] - pad_y) / scale
                kpts = row[6:].reshape(17, 3).copy()
                kpts[:, 0] = (kpts[:, 0] - pad_x) / scale
                kpts[:, 1] = (kpts[:, 1] - pad_y) / scale
                detections.append({
                    "box": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                    "score": round(score, 3),
                    "kpts": np.round(kpts, 1).tolist(),
                })
            return detections

        # Cas 2 : Modèle YOLO standard avec NMS -> shape (56, 8400)
        preds = raw.T if raw.shape[0] == 56 else raw
        scores = preds[:, 4]
        mask = scores >= CONF_THRES
        preds = preds[mask]
        scores = scores[mask]

        if len(preds) == 0:
            return detections

        cx, cy, w, h = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
        x1 = (cx - w / 2 - pad_x) / scale
        y1 = (cy - h / 2 - pad_y) / scale
        bw = w / scale
        bh = h / scale

        boxes_xywh = np.stack([x1, y1, bw, bh], axis=1).tolist()
        indices = cv2.dnn.NMSBoxes(boxes_xywh, scores.tolist(), CONF_THRES, IOU_THRES)

        if len(indices) > 0:
            for idx in np.array(indices).flatten():
                bx, by, bwidth, bheight = boxes_xywh[idx]
                kpts = preds[idx, 5:].reshape(17, 3).copy()
                kpts[:, 0] = (kpts[:, 0] - pad_x) / scale
                kpts[:, 1] = (kpts[:, 1] - pad_y) / scale
                detections.append({
                    "box": [round(bx, 1), round(by, 1), round(bx + bwidth, 1), round(by + bheight, 1)],
                    "score": round(float(scores[idx]), 3),
                    "kpts": np.round(kpts, 1).tolist(),
                })
        return detections


def recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def main():
    model = TrtYoloPose(ENGINE_PATH)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print(f"Serveur TensorRT en écoute sur {HOST}:{PORT}...")

    while True:
        conn, _ = srv.accept()
        try:
            while True:
                header = recv_exact(conn, 4)
                if not header:
                    break
                length = struct.unpack("!I", header)[0]
                jpg_bytes = recv_exact(conn, length)
                if not jpg_bytes:
                    break

                frame = cv2.imdecode(np.frombuffer(jpg_bytes, np.uint8), cv2.IMREAD_COLOR)
                dets = model.infer(frame) if frame is not None else []
                payload = json.dumps(dets).encode("utf-8")
                conn.sendall(struct.pack("!I", len(payload)) + payload)
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            conn.close()


if __name__ == "__main__":
    main()