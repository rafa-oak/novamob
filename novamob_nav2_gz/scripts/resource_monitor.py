import threading
import time
import psutil
from pynvml import *

class ResourceMonitor:
    def __init__(self, interval=1.0):
        self.interval = interval
        self.cpu_samples = []
        self.gpu_samples = []
        self.running = False
        self.thread = None

        # Init NVML for GPU access
        try:
            nvmlInit()
            self.gpu_handle = nvmlDeviceGetHandleByIndex(0)
        except NVMLError:
            self.gpu_handle = None
            print("[WARN] NVIDIA GPU not found or NVML init failed.")

    def _sample(self):
        while self.running:
            # CPU usage
            cpu = psutil.cpu_percent(interval=None)
            self.cpu_samples.append(cpu)

            # GPU usage (only if available)
            if self.gpu_handle:
                try:
                    util = nvmlDeviceGetUtilizationRates(self.gpu_handle)
                    self.gpu_samples.append(util.gpu)
                except NVMLError:
                    self.gpu_samples.append(0.0)
            else:
                self.gpu_samples.append(0.0)

            time.sleep(self.interval)

    def start(self):
        self.cpu_samples = []
        self.gpu_samples = []
        self.running = True
        self.thread = threading.Thread(target=self._sample)
        self.thread.start()
        print("[ResourceMonitor] Monitoring started.")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        print("[ResourceMonitor] Monitoring stopped.")

    def report(self):
        cpu_avg = sum(self.cpu_samples) / len(self.cpu_samples) if self.cpu_samples else 0
        gpu_avg = sum(self.gpu_samples) / len(self.gpu_samples) if self.gpu_samples else 0
        return cpu_avg, gpu_avg
