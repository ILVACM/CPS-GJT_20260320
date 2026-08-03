"""服务节点 程序入口。

启动引导由 system.bootstrap 模块承担（Step 4 实现），
本入口仅负责调用 bootstrap()。
"""
from system.bootstrap import bootstrap

if __name__ == "__main__":
    bootstrap()
