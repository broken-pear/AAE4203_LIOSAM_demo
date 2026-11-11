# =============================================================================
#  AAE4203 · LOAM vs LIO-SAM 对比实验
#
#  用法： make            （等同 make help，列出所有目标）
#         make check      （离线自检，不需要 ROS / Gazebo，约 50 秒）
#         make bench      （完整实验：6 个场景 × 2 个算法，约 35 分钟）
#         make deck       （生成英文 PPT）
#
#  这里只是把 README 里的命令收进一个入口，脚本本身一个都没搬 ——
#  所以 `bash experiments/run_bench.sh ...` 这些写法依然有效。
# =============================================================================

SHELL := /bin/bash
PY    ?= python3
ROS_SETUP ?= /opt/ros/jazzy/setup.bash

.DEFAULT_GOAL := help
.PHONY: help deps check build demo record replay bench list deck clean

help:  ## 列出所有目标（默认）
	@echo ''
	@echo '  AAE4203 · LOAM vs LIO-SAM 对比实验'
	@echo '  ------------------------------------------------'
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-8s\033[0m %s\n", $$1, $$2}'
	@echo ''
	@echo '  示例： make check      # 只要 50 秒，先确认代码没坏'
	@echo '         make bench      # 采数据（约 35 分钟，可中断，已录的包会复用）'
	@echo ''

deps:  ## 装依赖 + clone 两个上游算法 + 打补丁 + 编译（一次性）
	bash setup/install_deps.sh

check:  ## 七套离线自检（不启动 ROS，约 50 秒）
	PYTHON=$(PY) bash setup/check_all.sh

build:  ## 只重新编译 ros2_ws（改了代码之后用）
	source $(ROS_SETUP) && cd ros2_ws && colcon build --symlink-install

demo:  ## 启动 Demo 1（Gazebo + 小车 + RViz），Ctrl-C 退出
	source $(ROS_SETUP) && source ros2_ws/install/setup.bash && \
	  ros2 launch lio_sam_gz_bringup gazebo_lab1.launch.py

record:  ## 只录包（6 个场景），录完校验，不跑算法
	bash experiments/run_bench.sh --record-only --record

replay:  ## 复用已录的包，跑两条算法 + 评测 + 出表
	bash experiments/run_bench.sh --replay-only

bench:  ## 完整实验：录包 → 双算法回放 → 评测 → 体检 → 汇总 → 画图
	bash experiments/run_bench.sh

list:  ## 列出六个场景及其录包状态
	bash experiments/run_bench.sh --list

deck:  ## 用 results/tables/all_runs.csv 生成英文 PPT
	$(PY) report/build_deck.py

clean:  ## 删除生成物（录的包 / 原始轨迹 / 图 / 日志），保留指标表
	@echo '将删除：experiments/data/  results/raw/  results/logs/  results/figures/'
	@echo '保留  ：results/tables/  results/json/  results/manifest.json'
	rm -rf experiments/data results/raw results/logs results/figures
	@echo '完成。'
