import subprocess

# Define the commands to execute
commands = [
    "bash -c 'roslaunch main start.launch'",
    "bash -c 'python3 /home/ucar/Desktop/ucar/src/main/obstacle_avoidance.py'",
    "bash -c 'python3 /home/ucar/Desktop/ucar/src/main/t_dis.py'",
    "bash -c 'python3 /home/ucar/Desktop/ucar/src/main/navacation/navacation-super.py'"
]

# Function to execute a command
def run_command(command):
    subprocess.Popen(command, shell=True)

# Execute each command
for cmd in commands:
    run_command(cmd)
