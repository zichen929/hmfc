# IEEE TCST LaTeX 稿件

## 当前入口

- `main.tex`：整理后的英文稿件，基于 IEEE 官方 TCST Original Research LaTeX 模板。
- `main.pdf`：由当前源码编译的预览。
- `IEEEtran.cls`：官方文档类，保持原版。
- `bib.bib`：从 `../ccta26/bib.bib` 完整复制的参考文献数据库；仅排印正文引用的条目。
- `main.bbl`：BibTeX 生成的参考文献列表，供 LaTeX 排版时读取。
- `.latexmkrc`、`Makefile`：自动执行 pdfLaTeX、BibTeX 和必要的后续编译。
- `New_IEEEtran_how-to.pdf`、`New_IEEEtran_how-to.tex`：官方使用说明。
- `IEEE-Transactions-LaTeX2e-templates-and-instructions.zip`：保留的官方原始模板包，包含原始示例备份。

标题：**Immersion and Invariance Adaptive Motion–Force Control With an Unknown Planar Contact Surface**。

## 内容与来源

内容来自 `../TCST26E3.md`，初次整理日期为 2026-10-26；公式去重与交叉引用更新于 2026-10-27。原 Markdown 文件未修改。

正文依次为问题设定、Modeling、FIC 1、IAC 1.0.0 和主定理；补入 Reference Architecture、Error Coordinate、Nominal Motion-Force Control、2-D Surface Angle 以及 I&I 有效估计与参数流形定义。目标系统浸入、增广状态、浸入映射及动态匹配推导放入附录。

保留所选部分的控制律、稳定性推导及适用条件，调整标题层级、行文衔接和双栏公式断行。摘要和问题设定根据这些内容撰写。未纳入 FIC 2–4、IAC 1.0.1、数值例和备用材料；指向未纳入内容的内部链接已去除。重复使用的公式在首次出现处编号，后文通过 `\eqref` 引用；独立且不再引用的推导公式保持不编号。

Problem Setting 开头另复制了 `../ccta26/main-ccta26.tex` 的 Introduction 前三段，排除原最后一段贡献说明，并保留其中 18 条引用。去除了审阅注释与颜色标记，使用与源稿相同的 `cite` 包、BibTeX 和 `IEEEtran` 参考文献样式。

工作稿中的模板示例正文、示例参考文献、示例图片、虚构作者、卷期页眉、收稿日期及版权占位均已清除。作者信息尚未提供，因此当前不显示作者。正文保留 FIC 1 和 IAC 1.0.0 标识，便于与原笔记对应。

## 编译

在本目录运行以下命令，会自动生成参考文献并解析所有引用：

```bash
make
```

也可运行 `latexmk main.tex`。在仓库根目录运行时，使用 `make -C 04-writing/tcst26/latex`。

如果手动编译，必须执行完整顺序：

```bash
pdflatex -interaction=nonstopmode -halt-on-error main.tex
bibtex main
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
```

仅执行一次 `pdflatex main.tex`，在缺少 `main.bbl` 时会产生引用问号且没有文献列表。修改 `bib.bib` 或新增引用后，使用上述自动编译命令更新。

版式为 US Letter、10 pt、双栏 IEEE journal。已检查编译与 PDF 页面排版。

## 官方模板来源

模板下载于 2026-10-26。

- [TCST 作者指南](https://www.ieeecss.org/publication/transactions-control-systems-technology/author-info)
- [IEEE 模板选择器](https://template-selector.ieee.org/)
- [TCST 模板关联目录](https://template-selector.ieee.org/api/ieee-template-selector/template/publication-type/1/publication-title/186)
- [Original Research LaTeX 模板下载](https://template-selector.ieee.org/api/ieee-template-selector/template/377/download)
