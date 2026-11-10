# models/ — YOLO 权重（不随仓库发布）

本目录不收录模型权重文件（默认配置用 `yoloe-26s-seg.pt`，约 30 MB）。
两条获取途径：

1. **来源仓库的实测副本**（与本目录脚本一起验证过的版本）：

   ```bash
   curl -L -o apps/vision_grasp/models/yoloe-26s-seg.pt \
     https://gitee.com/gccodespace/d1arm_sdk/raw/grasp_back/calibration/models/yoloe-26s-seg.pt
   ```

2. **Ultralytics 官方发布**（同一模型，发布页 https://github.com/ultralytics/assets/releases ）；
   装了 `ultralytics` 后也可以不手动下载——首次 `YOLO("yoloe-26s-seg.pt")` 会自动
   下载到当前目录（注意进程按 CWD 解析，换目录运行要带上文件）。

模型文件版权归 Ultralytics，按其许可使用；本仓库只引用，不分发。
配置里换用其他实例分割模型（如 yolo11n-seg）改 `config/default.yaml` 的
`grasp.model` 即可，但开放词表提示词（`grasp.classes`）只有 yoloe/world 系列支持。
