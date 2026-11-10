"""路径解析：配置与数据产出统一放在 config/ 目录，模型在上级 models/。"""
from pathlib import Path


def data_root(config_path):
    """数据根目录 = 配置所在目录（config/）。
    hand_eye.npz、sessions/、示教文件与配置同目录存放。"""
    return Path(config_path).parent


def resolve_model(value, config_path):
    """YOLO 权重路径：相对路径相对标定根目录（config 的上级，即 models/
    所在目录）解析。"""
    path = Path(str(value))
    if path.is_absolute():
        return path
    return Path(config_path).parent.parent / path


def ensure_text_encoder(config_path):
    """保证 ultralytics 能找到 yoloe 文本编码器（mobileclip2_b.ts）。

    编码器放在 SDK 仓库根目录（config 上两级）；ultralytics 按进程 CWD
    解析该文件，若 CWD 没有副本/链接则建立指向仓库根副本的软链接
    （避免从其它目录运行时重复下载 ~240MB）。返回编码器实际路径；
    仓库根没有则返回 None（首次 set_classes 会自动下载到 CWD，
    之后建议挪到仓库根）。
    """
    target = Path(config_path).parents[2] / 'mobileclip2_b.ts'
    link = Path.cwd() / 'mobileclip2_b.ts'
    if not target.is_file():
        return None
    if not link.exists() and link.parent != target.parent:
        try:
            link.symlink_to(target)
        except OSError:
            pass  # CWD 无写权限等情况：交给 ultralytics 自行下载
    return target
