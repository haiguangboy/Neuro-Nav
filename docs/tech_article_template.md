# 技术论文长图文模板 (小红书/公众号适用)

## 适用场景
- 论文深度解读
- 技术教程系列
- 研究综述
- 需要长图分段发布的技术内容

## 设计风格

### 整体调性
- **底色**：纯白背景 (#fff)
- **主色**：纯黑文字 (#1a1a1a)
- **辅助色**：浅灰分隔 (#eee, #f5f5f5)
- **风格**：学术感、简洁、易读

### 字体方案
```css
/* 正文 */
font-family: -apple-system, BlinkMacSystemFont, "PingFang SC", "Hiragino Sans GB", "Microsoft YaHei", sans-serif;

/* 标题（如需衬线感） */
font-family: -apple-system, BlinkMacSystemFont, "PingFang SC", "Songti SC", serif;
```

### 字号规范
| 元素 | 字号 | 字重 |
|-----|------|-----|
| 大标题 | 36px | 700 |
| 章节标题 | 28px | 700 |
| 小节标题 | 20px | 600 |
| 正文 | 17px | 400 |
| 辅助文字 | 14-15px | 400 |

---

## 内容结构

```
封面页
├── 阅读时长提示
├── 大标题（问句/悬念）
└── 摘要（1-2句核心价值）

1. 问题背景
├── 现状痛点
├── 现有方案的缺陷
└── 引用框（核心矛盾）

2. 方法论
├── 2.1 核心创新点A
│   ├── 原理说明
│   └── 流程图
├── 2.2 核心创新点B
│   ├── 架构说明
│   └── 对比框

3. 实验结果
├── 数据高亮框（核心指标）
├── 对比表格
└── 关键发现

4. 消融实验（可选）
├── 各组件贡献
└── 关键结论引用框

5. 启示与展望
├── 行业意义（列表）
└── 局限性

标签区
└── #话题标签
```

---

## 组件样式

### 1. 封面页
```html
<div class="cover">
    <div class="reading-info">本文共XX字，阅读需X分钟</div>
    <h1 class="main-title">标题：问句/悬念式</h1>
    <p class="subtitle">一句话摘要，点明核心价值</p>
</div>
```

### 2. 章节标题
```html
<h2 class="section-title">1. 问题背景：XXX</h2>
<h3 class="subsection-title">2.1 小节标题 <span class="english">(English Name)</span></h3>
```

### 3. 高亮文本
```html
<span class="highlight">需要强调的内容</span>
<span class="term">专业术语</span>
<span class="english">(English Term)</span>
```

### 4. 列表
```html
<ul class="bullet-list">
    <li><span class="term">要点一</span>：详细说明...</li>
    <li><span class="term">要点二</span>：详细说明...</li>
</ul>
```

### 5. 引用框
```html
<div class="quote-box">
    核心观点或重要结论，用于强调关键信息。
</div>
```

### 6. 流程图
```html
<div class="flow-chart">
    <div class="flow-step">步骤1</div>
    <span class="flow-arrow">↓</span>
    <div class="flow-step">步骤2</div>
    <span class="flow-arrow">↓</span>
    <div class="flow-step">步骤3</div>
</div>
```

### 7. 对比框
```html
<div class="comparison-box">
    <div class="comparison-item">
        <div class="comparison-label">对比项A</div>
        <div>具体内容说明</div>
    </div>
    <div class="comparison-item">
        <div class="comparison-label">对比项B</div>
        <div>具体内容说明</div>
    </div>
</div>
```

### 8. 数据高亮
```html
<div class="data-highlight">
    <div class="data-row">
        <span class="data-label">指标名称</span>
        <span class="data-value">72.5%</span>
    </div>
    <div class="data-row">
        <span class="data-label">提升幅度</span>
        <span class="data-value">+2.1%</span>
    </div>
</div>
```

### 9. 表格
```html
<table class="simple-table">
    <tr>
        <th>列1</th>
        <th>列2</th>
        <th>列3</th>
    </tr>
    <tr>
        <td>数据1</td>
        <td>数据2</td>
        <td>数据3</td>
    </tr>
</table>
```

### 10. 标签区
```html
<div class="tags">
    <span class="tag">#标签1</span>
    <span class="tag">#标签2</span>
</div>
```

---

## 完整HTML模板

```html
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>文章标题</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, "PingFang SC", "Hiragino Sans GB", "Microsoft YaHei", sans-serif;
            background: #f5f5f5;
            color: #1a1a1a;
            line-height: 1.8;
        }

        .container {
            width: 100%;
            max-width: 100%;
            margin: 0 auto;
            background: #fff;
        }

        /* 封面页 */
        .cover {
            padding: 60px 50px;
            border-bottom: 1px solid #eee;
        }

        .reading-info {
            font-size: 14px;
            color: #888;
            margin-bottom: 30px;
        }

        .reading-info::before {
            content: "◎";
            margin-right: 8px;
        }

        .main-title {
            font-family: -apple-system, BlinkMacSystemFont, "PingFang SC", "Songti SC", serif;
            font-size: 36px;
            font-weight: 700;
            line-height: 1.4;
            margin-bottom: 30px;
            color: #1a1a1a;
        }

        .subtitle {
            font-size: 18px;
            color: #333;
            padding-left: 16px;
            border-left: 4px solid #1a1a1a;
            line-height: 1.6;
        }

        /* 内容区 */
        .section {
            padding: 50px;
        }

        .section-title {
            font-family: -apple-system, BlinkMacSystemFont, "PingFang SC", "Songti SC", serif;
            font-size: 28px;
            font-weight: 700;
            margin-bottom: 30px;
            color: #1a1a1a;
        }

        .subsection-title {
            font-size: 20px;
            font-weight: 600;
            margin: 35px 0 20px 0;
            padding-bottom: 10px;
            border-bottom: 1px solid #e0e0e0;
            color: #1a1a1a;
        }

        p {
            font-size: 17px;
            margin-bottom: 20px;
            text-align: justify;
        }

        .highlight {
            background: linear-gradient(to bottom, transparent 60%, #fff3cd 60%);
            padding: 0 2px;
        }

        .term {
            font-weight: 500;
        }

        .english {
            color: #666;
            font-size: 15px;
        }

        /* 列表 */
        .bullet-list {
            margin: 20px 0;
            padding-left: 0;
        }

        .bullet-list li {
            list-style: none;
            padding-left: 24px;
            position: relative;
            margin-bottom: 16px;
            font-size: 17px;
        }

        .bullet-list li::before {
            content: "●";
            position: absolute;
            left: 0;
            color: #1a1a1a;
        }

        /* 引用框 */
        .quote-box {
            border-left: 4px solid #1a1a1a;
            padding: 20px 25px;
            background: #f9f9f9;
            margin: 25px 0;
            font-size: 16px;
            color: #444;
        }

        /* 对比框 */
        .comparison-box {
            background: #fafafa;
            border-radius: 8px;
            padding: 25px;
            margin: 25px 0;
        }

        .comparison-item {
            margin-bottom: 20px;
        }

        .comparison-item:last-child {
            margin-bottom: 0;
        }

        .comparison-label {
            font-weight: 600;
            color: #333;
            margin-bottom: 8px;
        }

        /* 流程图 */
        .flow-chart {
            background: #f8f9fa;
            border-radius: 8px;
            padding: 30px;
            margin: 25px 0;
            text-align: center;
        }

        .flow-step {
            display: inline-block;
            background: #fff;
            border: 2px solid #1a1a1a;
            border-radius: 6px;
            padding: 12px 20px;
            margin: 8px;
            font-size: 15px;
            font-weight: 500;
        }

        .flow-arrow {
            display: block;
            font-size: 20px;
            color: #666;
            margin: 5px 0;
        }

        /* 数据高亮 */
        .data-highlight {
            background: #1a1a1a;
            color: #fff;
            padding: 20px 25px;
            border-radius: 8px;
            margin: 25px 0;
        }

        .data-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 10px 0;
            border-bottom: 1px solid rgba(255,255,255,0.2);
        }

        .data-row:last-child {
            border-bottom: none;
        }

        .data-label {
            font-size: 15px;
        }

        .data-value {
            font-size: 22px;
            font-weight: 700;
        }

        /* 表格 */
        .simple-table {
            width: 100%;
            border-collapse: collapse;
            margin: 25px 0;
            font-size: 15px;
        }

        .simple-table th,
        .simple-table td {
            border: 1px solid #ddd;
            padding: 12px 15px;
            text-align: left;
        }

        .simple-table th {
            background: #f5f5f5;
            font-weight: 600;
        }

        /* 分隔符 */
        .divider {
            height: 1px;
            background: #eee;
            margin: 0;
        }

        /* 脚注 */
        .footnote {
            font-size: 14px;
            color: #888;
            margin-top: 30px;
            padding-top: 20px;
            border-top: 1px solid #eee;
        }

        /* 标签 */
        .tags {
            padding: 40px 50px;
            background: #fafafa;
        }

        .tag {
            display: inline-block;
            background: #fff;
            border: 1px solid #ddd;
            border-radius: 4px;
            padding: 6px 12px;
            margin: 4px;
            font-size: 14px;
            color: #666;
        }
    </style>
</head>
<body>
    <div class="container">
        <!-- 封面页 -->
        <div class="cover">
            <div class="reading-info">本文共XX字，阅读需X分钟</div>
            <h1 class="main-title">【标题：问句/悬念式】</h1>
            <p class="subtitle">【一句话摘要】</p>
        </div>

        <!-- 正文区域 -->
        <div class="section">
            <h2 class="section-title">1. 章节标题</h2>
            <p>正文内容...</p>
        </div>

        <div class="divider"></div>

        <!-- 更多章节... -->

        <!-- 标签 -->
        <div class="tags">
            <span class="tag">#标签1</span>
            <span class="tag">#标签2</span>
        </div>
    </div>
</body>
</html>
```

---

## 使用说明

### 给Claude的提示词模板
```
请按照"技术论文长图文模板"，将这篇论文生成一个适合小红书发布的HTML长图文。

论文信息：
- 标题：XXX
- 核心创新点：XXX
- 关键数据：XXX

要求：
1. 自适应宽度，手机浏览器直接打开
2. 使用系统字体，确保iOS/Android兼容
3. 内容结构：问题→方案→效果→启示
```

### 发布流程
1. 下载HTML文件
2. 用iOS Edge / Android Chrome打开
3. 长截图或分段截图
4. 上传小红书

### 文字版介绍
 1. 还需要600字符以内高度概括，重点，创新点，亮点，价值点突出的文字介绍，用于在文字区介绍
 2. 文字不要加粗，段落之间空一行用于突出下一段的开头，可以使用图标，比如：对号，星号，工具，表情等
 3. 一段内为了凸显层次感，可以使用缩进，子内容的开头可以使用黑点引导
 4. 最后要总结这篇文字的价值，可以多视角，比如对于工程师，程序员意味着什么，对于创业者，投资人意味着什么，从读者的角度来挖掘价值

### 分割建议
- 每屏约800-900px高度
- 在章节标题处自然断开
- 首图包含封面+摘要，吸引点击

---

## 标签推荐（按领域）

### 具身智能/机器人
```
#具身智能 #机器人 #导航 #SLAM #世界模型 #强化学习 #机器人学习
```

### 大模型/AI
```
#大模型 #LLM #多模态 #VLM #深度学习 #人工智能 #AI论文
```

### 学术/会议
```
#NeurIPS #ICML #CVPR #ICLR #AAAI #论文解读 #顶会论文
```
