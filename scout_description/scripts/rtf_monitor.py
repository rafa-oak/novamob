import threading
import subprocess
import re

class RTFMonitor:
    def __init__(self, sim_env='fortress'):
        self.sim_env = sim_env  # 'fortress' or 'classic'
        self.rtf_samples = []
        self.running = False
        self.thread = None
        self.process = None

    def _parse_line(self, line):
        if self.sim_env == 'fortress':
            match = re.search(r"real_time_factor:\s*([0-9.]+)", line)
        else:  # classic gazebo
            match = re.search(r"Factor\[([0-9.]+)\]", line)

        if match:
            try:
                return float(match.group(1))
            except ValueError:
                pass
        return None

    def _sample(self):
        if self.sim_env == 'fortress':
            cmd = ['ign', 'topic', '-e', '-t', '/stats']
        else:  # classic gazebo
            cmd = ['gz', 'stats']

        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1
        )

        print(f"[RTFMonitor] Subprocess started for {self.sim_env}. Listening for stats...")

        for line in self.process.stdout:
            if not self.running:
                break
            rtf = self._parse_line(line)
            if rtf is not None:
                self.rtf_samples.append(rtf)

        self.process.stdout.close()
        self.process.wait()

    def start(self):
        self.rtf_samples = []
        self.running = True
        self.thread = threading.Thread(target=self._sample)
        self.thread.start()
        print("[RTFMonitor] Monitoring started.")

    def stop(self):
        self.running = False
        if self.process:
            self.process.terminate()
        if self.thread:
            self.thread.join()
        print("[RTFMonitor] Monitoring stopped.")

    def report(self):
        avg_rtf = sum(self.rtf_samples) / len(self.rtf_samples) if self.rtf_samples else 0.0
        return avg_rtf
