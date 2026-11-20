# models/ — YOLO 权重（不随仓库发布）

本目录不收录模型权重文件（默认配置用 `yoloe-26s-seg.pt`，约 30 MB）。
获取方式：

1. **来源仓库的副本**（与本目录脚本一起验证过的版本）。注意 gitee 对 raw
   大文件直链返回 403，用 git 协议克隆后拷贝：

   ```bash
   git clone -b grasp_back https://gitee.com/gccodespace/d1arm_sdk.git /tmp/d1arm_src
   cp /tmp/d1arm_src/calibration/models/yoloe-26s-seg.pt apps/vision_grasp/models/
   ```

2. **Ultralytics 官方发布**（同一模型，发布页 https://github.com/ultralytics/assets/releases ）；
   装了 `ultralytics` 后也可以不手动下载——首次 `YOLO("yoloe-26s-seg.pt")` 会自动
   下载到当前目录（注意进程按 CWD 解析，换目录运行要带上文件）。

权重到位校验（可选）：`sha256sum apps/vision_grasp/models/yoloe-26s-seg.pt`，
与来源仓库克隆副本一致即可。

模型文件版权归 Ultralytics，按其许可使用；本仓库只引用，不分发。
配置里换用其他实例分割模型（如 yolo11n-seg）改 `config/default.yaml` 的
`grasp.model` 即可，但开放词表提示词（`grasp.classes`）只有 yoloe/world 系列支持。
