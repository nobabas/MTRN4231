import subprocess

def run_camera():
    command = 'ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:=true'
    process = subprocess.Popen(command, shell=True)
    return process

def stop_camera(process):
    if process:
        process.terminate()
        process.wait()

def main(args=None):
    run_camera()

if __name__ == '__main__':
    main()