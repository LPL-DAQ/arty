from cx_Freeze import setup
import os

VERSION = '0.1'

build_exe_options = {
    "includes": ["google.protobuf", "google", "flasherd_pb2", "flasherd_pb2_grpc"],
}

bdist_msi_options = {
    "add_to_path": True,
    "all_users": True,
    "launch_on_finish": True,
    "target_name": "flasherd"
}

server_script_path = f'{os.path.dirname(os.path.realpath(__file__))}/server.py'
setup(
    name="flasherd",
    version=VERSION,
    options={"build_exe": build_exe_options, "bdist_msi": bdist_msi_options},
    executables=[{
        "script": server_script_path, 
        "base": "console",
        "shortcut_name": "flasherd",
        "shortcut_dir": "StartMenuFolder",
        "target_name": "flasherd"
    }],
)
