import subprocess
import sys
import time

def main():
    proc = subprocess.Popen(
        ['ros2', 'launch', 'post_tests', 'receiver_sender_test_launch.py'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    received_count = 0
    timeout_sec = 10
    start_time = time.time()

    try:
        while True:
            line = proc.stdout.readline()
            if not line:
                break

            print(line, end='')  # Optional: print live logs

            # Detect parcel received logs from receiver station
            if "receiver_test" in line and "Received parcel:" in line:
                received_count += 1

            # Stop early if all parcels received
            if received_count >= 3:
                proc.terminate()
                print("TEST PASSED: Receiver got all parcels.")
                return 0

            # Timeout safety
            if time.time() - start_time > timeout_sec:
                print(f"TEST FAILED: Timeout waiting for parcels. Received {received_count}/3")
                proc.terminate()
                proc.wait()
                return 1

    except KeyboardInterrupt:
        proc.terminate()
        return 1

    proc.wait()
    if received_count >= 3:
        print("TEST PASSED: Receiver got all parcels.")
        return 0
    else:
        print(f"TEST FAILED: Only received {received_count} parcels")
        return 1

if __name__ == "__main__":
    sys.exit(main())
