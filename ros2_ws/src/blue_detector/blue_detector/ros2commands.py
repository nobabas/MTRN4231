import subprocess

def run_static_transform():
    """Run the static transform publisher command"""
    command = "ros2 run tf2_ros static_transform_publisher 1.30317 0.0174152 0.675776 -0.388123 -0.0054127 0.92155 0.0087602 camera_link base"
    
    process = subprocess.Popen(command, shell=True)
    return process

def stop_static_transform(process):
    """Stop the running transform publisher"""
    if process:
        process.terminate()
        process.wait()