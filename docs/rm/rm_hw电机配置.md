# rm_hw电机配置
要实现电机和urdf中acutor的映射，需要用到rm_control中的rm_hw，只需要在rm_config编写配置文件并用launch导入并启动rm_hw即可

```yaml
rm_hw:
  bus:
    - can0
    # can0 can1 ...
  loop_frequency: 1000
  cycle_time_error_threshold: 0.001
  thread_priority: 95
  # Configurations of the actuators
  actuators:
    bar_motor:
      bus: can0
      id: 0x201 
      type: rm_2006
      lp_cutoff_frequency: 50
```

