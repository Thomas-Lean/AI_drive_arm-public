"""推理设备选择：配置 grasp.device 填 auto 时按本机实际情况选。"""


def resolve_device(value):
    """'auto' → 有可用 CUDA 显卡用 'cuda:0'，否则 'cpu'；其他值（cpu / cuda:0 …）原样返回。"""
    if str(value).strip().lower() != 'auto':
        return value
    try:
        import torch
        return 'cuda:0' if torch.cuda.is_available() else 'cpu'
    except Exception:  # noqa: BLE001 — 没装 torch 或驱动异常时退回 CPU
        return 'cpu'
