import numpy as np
import cv2

class HailoYolov8Pose:
    def __init__(self, hef_path):
        self.hef_path = hef_path
        
        # We will attempt to import hailort. If it fails, we provide a mock for now
        # so the node doesn't crash if HailoRT is not installed on the dev machine.
        try:
            from hailo_platform import VDevice
            self.hailort_available = True
            
            self.params = VDevice.create_params()
            self.vdevice = VDevice(self.params)
            self.infer_model = self.vdevice.create_infer_model(hef_path)
            self.infer_model.set_batch_size(1)
            
            self.configured_infer_model = self.infer_model.configure()
            self.bindings = self.configured_infer_model.create_bindings()
            
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
        # Note: True post-processing requires parsing the exact output layers from the HEF.
        # This is a simplified wrapper that users can adapt based on their specific HEF's NMS configuration.
        
        outputs = {name: self.bindings.output(name).get_buffer() for name in self.bindings.output_names()}
        
        return self._post_process(outputs, image.shape, conf)

    def _post_process(self, outputs, orig_shape, conf_thresh):
        # Print the raw tensor shapes so we know exactly how to parse this specific HEF file
        print("\n" + "="*50)
        print("HAILO RAW OUTPUT TENSORS:")
        for name, tensor in outputs.items():
            print(f" - {name}: shape={tensor.shape}, dtype={tensor.dtype}")
        print("="*50 + "\n")
        
        # Placeholder for actual YOLOv8 pose post-processing from Hailo raw tensors
        # Since Hailo HEF outputs depend on the exact model zoo compilation (often containing NMS),
        # we return a mocked result structure compatible with Ultralytics for integration testing.
        return self._mock_predict(np.zeros(orig_shape, dtype=np.uint8))

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
