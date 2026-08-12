# Phase 0 工程备份清单

记录日期：2026-08-12（Asia/Shanghai）
可恢复基线提交：`b6b7ab75ff5434b1ad739d55735c681d428ba4ea`

## Git 对象

| 文件 | 基线提交中的 Git blob | 当前工作树 Git blob | 当前工作树 SHA-256 |
|---|---|---|---|
| `aethor_robo_v1/CtrBoard-H7_FDCAN.ioc` | `fa8a180d470ab27354b1254cd0653077990deab2` | `fa8a180d470ab27354b1254cd0653077990deab2` | `559B6637E3B2031F108A047F768E6D099D331C4A25B03F448C9A448CB1785AE6` |
| `aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx` | `042d0b05d9a979a929d2ea81e6ef896f0ddca558` | `8c950ad8ff0696f6ce8543d74f7cde036b582c2a` | `6A42B41BD0712E0ED0E007F1961F2045F8FA177F95BDFFB2E3057871E31EFF33` |

`uvprojx` 的两个 blob 不同，原因是执行前已有未提交的 `<LayerInfo>` 用户元数据。基线提交中的对象和用户工作树对象都已分别记录，Phase 0 不把二者混为同一版本。

## 无覆盖导出

在 Git 根目录 `E:/Desktop_E/TCG/Aethor_robo_fw` 执行以下命令，可把基线工程导出为独立 ZIP，不覆盖当前工程：

```powershell
git archive --format=zip --output='phase00-project-baseline-b6b7ab7.zip' b6b7ab75ff5434b1ad739d55735c681d428ba4ea -- aethor_robo_v1/CtrBoard-H7_FDCAN.ioc aethor_robo_v1/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx
```

导出后可用以下命令核验 ZIP 是否包含两个目标文件：

```powershell
tar -tf '.\phase00-project-baseline-b6b7ab7.zip'
```

恢复时应先解压到独立临时目录并比较，不直接覆盖工作树文件。
