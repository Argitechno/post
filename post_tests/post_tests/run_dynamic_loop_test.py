import subprocess
import sys
import time

def main():
    proc = subprocess.Popen(
        ['ros2', 'launch', 'post_tests', 'dynamic_loop_test_launch.py'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    ttl_expired_detected = False
    timeout_sec = 15  # Give some extra time for the parcel to loop & expire
    start_time = time.time()

    try:
        while True:
            line = proc.stdout.readline()
            if not line:
                break

            print(line, end='')  # Optional: print live logs

            # Detect TTL expired log line
            if "TTL expired, dropping." in line:
                ttl_expired_detected = True
                proc.terminate()
                print("TEST PASSED: Parcel TTL expired as expected.")
                return 0

            # Timeout safety
            if time.time() - start_time > timeout_sec:
                print("TEST FAILED: Timeout waiting for TTL expired message.")
                proc.terminate()
                return 1

    except KeyboardInterrupt:
        proc.terminate()
        return 1

    proc.wait()
    if ttl_expired_detected:
        print("TEST PASSED: Parcel TTL expired as expected.")
        return 0
    else:
        print("TEST FAILED: TTL expired message not detected.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
