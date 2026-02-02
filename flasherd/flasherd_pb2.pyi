from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Arg(_message.Message):
    __slots__ = ("regular", "binary")
    REGULAR_FIELD_NUMBER: _ClassVar[int]
    BINARY_FIELD_NUMBER: _ClassVar[int]
    regular: str
    binary: str
    def __init__(self, regular: _Optional[str] = ..., binary: _Optional[str] = ...) -> None: ...

class RunCommandRequest(_message.Message):
    __slots__ = ("command_windows", "command_macos", "command_linux", "args", "stdin", "binary_name", "binary_chunk")
    COMMAND_WINDOWS_FIELD_NUMBER: _ClassVar[int]
    COMMAND_MACOS_FIELD_NUMBER: _ClassVar[int]
    COMMAND_LINUX_FIELD_NUMBER: _ClassVar[int]
    ARGS_FIELD_NUMBER: _ClassVar[int]
    STDIN_FIELD_NUMBER: _ClassVar[int]
    BINARY_NAME_FIELD_NUMBER: _ClassVar[int]
    BINARY_CHUNK_FIELD_NUMBER: _ClassVar[int]
    command_windows: str
    command_macos: str
    command_linux: str
    args: _containers.RepeatedCompositeFieldContainer[Arg]
    stdin: bytes
    binary_name: str
    binary_chunk: bytes
    def __init__(self, command_windows: _Optional[str] = ..., command_macos: _Optional[str] = ..., command_linux: _Optional[str] = ..., args: _Optional[_Iterable[_Union[Arg, _Mapping]]] = ..., stdin: _Optional[bytes] = ..., binary_name: _Optional[str] = ..., binary_chunk: _Optional[bytes] = ...) -> None: ...

class RunCommandResponse(_message.Message):
    __slots__ = ("exit_code", "stdout", "stderr")
    EXIT_CODE_FIELD_NUMBER: _ClassVar[int]
    STDOUT_FIELD_NUMBER: _ClassVar[int]
    STDERR_FIELD_NUMBER: _ClassVar[int]
    exit_code: int
    stdout: bytes
    stderr: bytes
    def __init__(self, exit_code: _Optional[int] = ..., stdout: _Optional[bytes] = ..., stderr: _Optional[bytes] = ...) -> None: ...
