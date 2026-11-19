from __future__ import annotations

from dataclasses import dataclass

from tinygrad import Context
from tinygrad.codegen import to_program
from tinygrad.device import Device
from tinygrad.engine.jit import prune_linear
from tinygrad.engine.realize import run_linear
from tinygrad.uop.ops import GroupOp, Ops, UOp
from tinygrad.dtype import dtypes

from .offline_buffers import descriptor
from .offline_model import InitializedBuffer, OfflineModel
from .schema import ExportError


@dataclass(frozen=True, slots=True)
class PreparedSchedule:
    linear: UOp
    initialized: tuple[InitializedBuffer, ...]
    onetime_calls: int


def prepare_constants(model: OfflineModel, linear: UOp) -> PreparedSchedule:
    kept, onetime = prune_linear(linear, {tensor.uop.buf_uop for tensor in model.bindings.inputs.values()})
    initialized = {item.buffer: item.data for item in model.initialized}
    host: dict[UOp, UOp] = {}
    known = set(initialized)
    for call in onetime.src:
        if call.op is not Ops.CALL or call.src[0].op not in {Ops.SINK, Ops.COPY}:
            raise ExportError("unsupported offline constant operation")
        if any(value.op in GroupOp.ALU - {Ops.WHERE} and value.dtype.scalar() in dtypes.floats for value in call.src[0].toposort()):
            raise ExportError("offline constant transform includes unaudited floating arithmetic")
        arguments = call.src[1:]
        for argument in arguments:
            if argument.op is not Ops.BUFFER:
                raise ExportError("offline constant transform requires direct buffer storage")
            if argument not in host:
                host[argument] = UOp.new_buffer("CPU", argument.arg, argument.dtype)
                original = descriptor(argument)
                if argument not in initialized and original.is_allocated():
                    initialized[argument] = bytes(original.as_memoryview())
                    known.add(argument)
                if argument in initialized:
                    if len(initialized[argument]) != descriptor(argument).nbytes:
                        raise ExportError("offline initializer size differs from storage")
                    host[argument].buffer.ensure_allocated().copyin(memoryview(bytearray(initialized[argument])))
                else:
                    # A weight transform can leave gaps between image rows.
                    # These bytes are serialized along with the written values.
                    host[argument].buffer.ensure_allocated().copyin(memoryview(bytearray(original.nbytes)))
        with Context(DEV="CPU:LLVM", IMAGE=1, FLOAT16=1, NOLOCALS=1, BEAM=0):
            if call.src[0].op is Ops.COPY:
                program, outputs, inputs = call.src[0], (0,), (1,)
            else:
                program = to_program(call.src[0], Device["CPU"].renderer)
                outputs, inputs = program.arg.outs, program.arg.ins
            if any(arguments[index] not in known for index in inputs):
                raise ExportError("offline constant transform reads unknown data")
            host_call = call.replace(src=(program, *(host[argument] for argument in arguments)))
            run_linear(UOp(Ops.LINEAR, src=(host_call,)), jit=True, wait=True)
        for index in outputs:
            original = arguments[index]
            known.add(original)
            initialized[original] = bytes(host[original].buffer.as_memoryview())
    return PreparedSchedule(kept, tuple(InitializedBuffer(buffer, data) for buffer, data in initialized.items()), len(onetime.src))
