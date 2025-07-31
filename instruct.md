这些指令是用于控制机器人并记录数据的命令行工具。以下是每个指令的详细解释：

校准机器人：


bash
python lerobot/scripts/control_robot.py --robot.type=so100 --control.type=calibrate
该命令用于校准机器人，确保其处于正确的工作状态。
无限遥操作（最高频率）：


bash
python lerobot/scripts/control_robot.py --robot.type=so100 --robot.cameras='{}' --control.type=teleoperate
该命令允许用户以最高频率（约200 Hz）远程控制机器人，直到按下CTRL+C退出。
无限遥操作（限制频率）：


bash
python lerobot/scripts/control_robot.py --robot.type=so100 --control.type=teleoperate --control.fps=30
该命令允许用户以限制的频率（30 Hz）远程控制机器人，模拟数据记录时的频率。
记录一个episode以测试回放：


bash
python lerobot/scripts/control_robot.py --robot.type=so100 --control.type=record --control.fps=30 --control.single_task="Grasp a lego block and put it in the bin." --control.repo_id=$USER/koch_test --control.num_episodes=1 --control.push_to_hub=True
该命令记录一个episode（任务执行过程），并将其推送到Hugging Face Hub，以便后续测试回放。
可视化数据集：


bash
python lerobot/scripts/visualize_dataset.py --repo-id $USER/koch_test --episode-index 0
该命令用于可视化已记录的数据集，查看特定episode的执行情况。
回放测试episode：


bash
python lerobot/scripts/control_robot.py replay --robot.type=so100 --control.type=replay --control.fps=30 --control.repo_id=$USER/koch_test --control.episode=0
该命令用于回放之前记录的episode，验证记录的准确性。
记录完整数据集以训练策略：


bash
python lerobot/scripts/control_robot.py record --robot.type=so100 --control.type=record --control.fps 30 --control.repo_id=$USER/koch_pick_place_lego --control.num_episodes=50 --control.warmup_time_s=2 --control.episode_time_s=30 --control.reset_time_s=10
该命令记录多个episode，用于训练机器人策略。每个episode包括2秒的预热时间、30秒的记录时间和10秒的环境重置时间。
远程控制机器人（如LeKiwi）：


bash
python lerobot/scripts/control_robot.py --robot.type=lekiwi --control.type=remote_robot
该命令用于在远程设备（如Raspberry Pi）上运行，控制远程机器人。
键盘控制数据记录流程：

使用键盘控制数据记录流程，如提前退出、重新记录当前episode或停止记录。
继续/恢复数据记录：


bash
--control.resume=true
该选项允许用户继续之前未完成的数据记录。
使用ACT策略训练数据集：


bash
python lerobot/scripts/train.py --dataset.repo_id=${HF_USER}/koch_pick_place_lego --policy.type=act --output_dir=outputs/train/act_koch_pick_place_lego --job_name=act_koch_pick_place_lego --device=cuda --wandb.enable=true
该命令使用ACT策略训练机器人，并将训练结果保存到指定目录。
在机器人上运行预训练策略：


bash
python lerobot/scripts/control_robot.py --robot.type=so100 --control.type=record --control.fps=30 --control.single_task="Grasp a lego block and put it in the bin." --control.repo_id=$USER/eval_act_koch_pick_place_lego --control.num_episodes=10 --control.warmup_time_s=2 --control.episode_time_s=30 --control.reset_time_s=10 --control.push_to_hub=true --control.policy.path=outputs/train/act_koch_pick_place_lego/checkpoints/080000/pretrained_model
该命令在机器人上运行预训练的ACT策略，并记录执行过程。
这些指令提供了从校准、控制、记录到训练和回放的完整工作流程，帮助用户有效地管理和操作机器人。


python lerobot/scripts/train.py   --dataset.repo_id=Wanacola/koch_test   --policy.type=act   --output_dir=outputs/train/act_so100_test   --job_name=act_lekiwi_test   --policy.device=cuda


python lerobot/scripts/train.py  --dataset.root=/home/cheku/.cache/huggingface/lerobot --dataset.repo_id=Wanacola/koch_test --policy.type=act --output_dir=outputs/train/act_so100_test --job_name=act_so100_test --policy.device=cuda --wandb.enable=false

python lerobot/scripts/control_robot.py --robot.type=lekiwi --control.type=record --control.fps=30 --control.single_task="Grasp a lego block and put it in the bin." --control.repo_id=Wanacola/koch_test --control.num_episodes=2 --control.push_to_hub=true  --control.episode_time_s=30 --control.reset_time_s=10 


想喝可乐