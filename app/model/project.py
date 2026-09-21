"""计算模型(.trans_model) 的 JSON 持久化。

负责模型的读写与文件后缀规整；字段的组装/还原由界面层完成
（取决于当前表单控件状态，属于视图层职责）。
"""
import json

from config import MODEL_SUFFIX


def load_model(path):
    """读取 .trans_model 模型文件并反序列化为 dict。"""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_model(data, path):
    """把模型字典写入文件（UTF-8、保持中文、缩进）。"""
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def ensure_suffix(path):
    """若路径缺少模型后缀则自动补全。"""
    if not path.endswith(MODEL_SUFFIX):
        return path + MODEL_SUFFIX
    return path