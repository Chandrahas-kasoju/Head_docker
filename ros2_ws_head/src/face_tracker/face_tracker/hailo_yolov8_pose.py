import numpy as np
import cv2

class HailoYolov8Pose:
    def __init__(self, hef_path):
        self.hef_path = hef_path
        
        # We will attempt to import hailort. If it fails, we provide a mock for now
        # so the node doesn't crash if HailoRT is not installed on the dev machine.
        try:
            import hailo_platform
            from hailo_platform import VDevice
            self.hailort_available = True
            
            self.params = VDevice.create_params()
            self.params.scheduling_algorithm = hailo_platform.HailoSchedulingAlgorithm.NONE
            self.vdevice = VDevice(self.params)
            self.infer_model = self.vdevice.create_infer_model(hef_path)
            self.infer_model.set_batch_size(1)
            
            # Configure all output streams to be FLOAT32 so we can easily allocate numpy arrays for them
            for output_name in self.infer_model.output_names:
                self.infer_model.output(output_name).set_format_type(hailo_platform.FormatType.FLOAT32)
            
            self.configured_infer_model = self.infer_model.configure()
            self.configured_infer_model.activate()
            self.bindings = self.configured_infer_model.create_bindings()
            
            # Pre-allocate output buffers since get_buffer_as_view fails if not configured
            self.output_buffers = {}
            for output_name in self.infer_model.output_names:
                tensor_shape = self.infer_model.output(output_name).shape
                self.output_buffers[output_name] = np.empty(tensor_shape, dtype=np.float32)
                self.bindings.output(output_name).set_buffer(self.output_buffers[output_name])
            
        except ImportError as e:
            import sys
            print(f"WARNING: hailort could not be imported. Reason: {e}")
            print(f"sys.path: {sys.path}")
            print("Running in mock mode.")
            self.hailort_available = False

    def __call__(self, image, conf=0.5, **kwargs):
        return self.predict(image, conf)

    def predict(self, image, conf=0.5):
        if not self.hailort_available:
            return self._mock_predict(image)

        # Pre-process
        img_resized = cv2.resize(image, (640, 640))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        input_data = np.expand_dims(img_rgb, axis=0).astype(np.uint8)

        # Infer
        self.bindings.input().set_buffer(input_data)
        self.configured_infer_model.run([self.bindings], 1000)
        
        # Extract output tensors
        # YOLOv8 pose outputs multiple tensors (for boxes and keypoints)
        # For simplicity, we assume the DFC exported a combined output tensor or 
        # we parse the raw tensors.
        
        outputs = self.output_buffers
        
        return self._post_process(outputs, image.shape, conf)

    def _post_process(self, outputs, orig_shape, conf_thresh):
        # We need to find the DFL, Cls, and Kpt tensors for each stride.
        # Group them by spatial shape (e.g. 80x80, 40x40, 20x20)
        groups = {}
        for name, tensor in outputs.items():
            h, w, c = tensor.shape
            if h not in groups:
                groups[h] = {}
            if c == 64:
                groups[h]['dfl'] = tensor
            elif c == 1:
                groups[h]['cls'] = tensor
            elif c == 51:
                groups[h]['kpt'] = tensor
                
        all_boxes = []
        all_scores = []
        all_kpts = []
        
        for h in sorted(groups.keys(), reverse=True): # 80, 40, 20
            stride = 640 / h
            dfl = groups[h]['dfl'].reshape(-1, 64)
            cls = groups[h]['cls'].reshape(-1)
            kpt = groups[h]['kpt'].reshape(-1, 17, 3)
            
            # 1. Sigmoid class scores
            cls_scores = 1 / (1 + np.exp(-cls))
            
            # Filter by conf early to save computation
            mask = cls_scores > conf_thresh
            if not np.any(mask):
                continue
                
            dfl = dfl[mask]
            cls_scores = cls_scores[mask]
            kpt = kpt[mask]
            
            # Generate grid for these specific masked pixels
            grid_y, grid_x = np.mgrid[0:h, 0:h]
            grid = np.stack((grid_x, grid_y), axis=-1).reshape(-1, 2)
            grid = grid[mask]
            
            # 2. Decode DFL boxes
            dfl = dfl.reshape(-1, 4, 16)
            e_x = np.exp(dfl - np.max(dfl, axis=-1, keepdims=True))
            prob = e_x / e_x.sum(axis=-1, keepdims=True)
            weight = np.arange(16, dtype=np.float32)
            dist = np.sum(prob * weight, axis=-1)
            
            # dist is (left, top, right, bottom)
            x1 = grid[:, 0] - dist[:, 0]
            y1 = grid[:, 1] - dist[:, 1]
            x2 = grid[:, 0] + dist[:, 2]
            y2 = grid[:, 1] + dist[:, 3]
            
            # OpenCV NMS requires [x, y, w, h] format
            w = x2 - x1
            h_box = y2 - y1
            boxes = np.stack([x1, y1, w, h_box], axis=-1) * stride
            
            # 3. Decode Keypoints
            kpt[:, :, 0] = (kpt[:, :, 0] * 2.0 + grid[:, 0:1] - 0.5) * stride
            kpt[:, :, 1] = (kpt[:, :, 1] * 2.0 + grid[:, 1:2] - 0.5) * stride
            kpt[:, :, 2] = 1 / (1 + np.exp(-kpt[:, :, 2])) # sigmoid vis
            
            all_boxes.append(boxes)
            all_scores.append(cls_scores)
            all_kpts.append(kpt)
            
        if not all_boxes:
            return []
            
        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        kpts = np.concatenate(all_kpts, axis=0)
        
        # NMS
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), conf_thresh, 0.45)
        
        results = []
        if len(indices) > 0:
            for i in indices.flatten():
                box = boxes[i]
                scale_x = orig_shape[1] / 640.0
                scale_y = orig_shape[0] / 640.0
                
                x1 = int(box[0] * scale_x)
                y1 = int(box[1] * scale_y)
                w = int(box[2] * scale_x)
                h_box = int(box[3] * scale_y)
                
                scaled_box = [x1, y1, x1 + w, y1 + h_box]
                
                scaled_kpts = []
                for kpt_idx in range(17):
                    x = kpts[i, kpt_idx, 0] * scale_x
                    y = kpts[i, kpt_idx, 1] * scale_y
                    vis = kpts[i, kpt_idx, 2]
                    scaled_kpts.append((int(x), int(y), float(vis)))
                    
                results.append({
                    "bbox": scaled_box,
                    "score": float(scores[i]),
                    "keypoints": scaled_kpts
                })
                
        return results

    def _mock_predict(self, image):
        class MockTensor:
            def __init__(self, arr):
                self.arr = arr
            def cpu(self):
                return self
            def numpy(self):
                return self.arr
            def __getitem__(self, idx):
                return MockTensor(self.arr[idx])

        class MockBox:
            def __init__(self):
                self.xyxy = MockTensor(np.array([[10, 10, 100, 100]]))
                self.conf = MockTensor(np.array([0.9]))

        class MockKeypoints:
            def __init__(self):
                self.xy = MockTensor(np.array([np.zeros((17, 2))]))
                self.conf = MockTensor(np.array([np.ones(17)]))

        class MockResult:
            def __init__(self):
                self.boxes = [MockBox()]
                self.keypoints = MockKeypoints()
            
            def plot(self, img=None):
                return img if img is not None else np.zeros((640,640,3), dtype=np.uint8)
                
            def __getitem__(self, idx):
                return self

        return [MockResult()]
