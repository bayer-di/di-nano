# di-nano

该项目是 `DI-NANO`, 使用最优美语言 `Python` 大法炮制而成, 它的角色是`ROS2 NODE`, 顾名思义, 其构建在[`ROS2(Robot Operating System)`](https://docs.ros.org/)基础环境下。

其主要职责:

- `AGV` 与 `机器人调度系统` 的 `黏合剂`;
- `AGV` 与  `路网编辑、路网规划系统` 的 `承重梁`;
- 上行 `AGV` 所有`ROS2 NODE Data` 至 `Mqtt Server`;
- 下行 `机器人调度系统` `路网编辑、路网规划系统` 的指令;
- TODO (视频、音频、照片、环境等）数据中转站;
  
> `Data` 包含: `下行指令`、`下行任务调度指令`、`上行AGV车辆状态数据`、`上行任务状态`、`上行通用数据` 等等...

- ...

# 文件介绍

```shell
 - di-nano
 |-- devops    # devops
 |-- nano      # 真实的项目
 |-- nano-msg  # 自定义ROS2消息 (Msg、Service、Action)
```

# ROS  相关文档

- [ROS2 文档](https://docs.ros.org/)
- [ROS2 官网](https://ros.org/)

# 本地开发

所有操作均在 `di-nano` 目录

## 启动 EMQX

```shell
docker-compose -f ./devops/docker-compose-emqx.yml up -d
```

## 第一次初始化项目

```shell
cd nano/nano

python3 -m venv .venv

source .venv/bin/activate

pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt
```

## 非第一次初始化

```shell
cd nano/nano

source .venv/bin/activate

pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt
```

## [可选] 安装新的Py包

```shell
cd nano/nano

pip freeze > requirements.txt
```

# ROS 环境下调试

## 构建 `ros2:emqx` 基本环境(内部安装)

所有操作均在 `di-nano` 目录

```shell
docker build -t ros2:emqx -f devops/ros.dockerfile .
```

## 启动一个工作区

所有操作均在 `di-nano` 目录

```shell
docker run --name ros2 \
    -v .:/root/workspaces/src \
    -itd ros2:emqx bash
```

## [可选] 执行测试 `ROS2` 容器是否正常

开启两个终端执行分别执行下面命令

- 第一个终端 `监听node`

    ```shell
    docker exec -it nano_dev ./ros_entrypoint.sh bash
    
    ros2 run demo_nodes_cpp listener
    ```

- 第二个终端 `发布node`

    ```shell
    docker exec -it nano_dev ./ros_entrypoint.sh bash
    
    ros2 run demo_nodes_cpp talker
    ```

## [可选]其他指令合集

```shell
# 创建ROS2包（在工作区间`src`）`<package_name>`为包名
ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy
```

```shell
# 构建和安装包，进入工作空间目录，执行构建和安装
colcon build --packages-select <package_name>
# 或者
colcon build
```

```shell
# 激活安装
source install/setup.bash
# 运行节点
ros2 run <package_name> <node_name>

# 或者以 launch 方式运行
ros2 launch <package_name> <package_name>.launch.py
```

# 傻瓜式调试开发

## 启动容器

- 第一种单独构建镜像然后启动

    ```shell
    # 先进入 `di-nano/` 目录
    cd di-nano/

    # 执行挂载
    docker run --name ros2 -v .:/root/workspaces/src -itd ros2:emqx /bin/bash
    ```

- 第二种用 docker-compose 启动

    ```shell
    # 先进入 `di-nano/devops` 目录
    docker-compose -f ./devops/docker-compose-ros.yml up -d
    ```

## 后续调试开发步骤

```shell
# 开启终端进入容器 或者 使用  VSCODE REMOTE 模式
docker exec -it nano_dev ./ros_entrypoint.sh bash

# 进入工作区
cd ~/workspaces/

# 批量安装依赖的Py包
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r src/nano/nano/requirements.txt 

# 构建 nano 包
# colcon build --packages-select nano
colcon build

# 安装 nano 包
source install/setup.bash

# 运行 nano 包
ros2 run nano nano

# 非 ~/workspaces/src/ 下执行, 指定配置文件参数
ros2 run nano nano --ros-args -p conf_file:=/root/workspaces/src/nano/params/nano.yaml

# 以 launch 方式启动, 要佩戴上  start  参数
ros2 launch nano nano.launch.py start:=true

```

# 知识点

- EMQX 系统主题相关
  
  以 `$SYS/broker/{node}/` 开头 , `{node}`为产生该事件或者节点的名称

  > - `$SYS/brokers` `集群节点列表`
  > - `$SYS/broker/${node}/version` `EMQX 版本`
  > - `$SYS/broker/${node}/uptime`   `EMQX 运行时间`
  > - `$SYS/broker/${node}/datetime` `EMQX 系统时间`
  > - `$SYS/broker/${node}/sysdescrv` `EMQX 描述`
  > - `$SYS/broker/${node}/clients/${client_id}/connected`    `客户端上线`
  > - `$SYS/broker/${node}/clients/${client_id}/disconnected` `客户端下线`

- EMQX 管理界面
  
  > [http://localhost:18083/](http://localhost:18083/) (admin/public)

- `nano` restful api地址
  
  > - redoc <http://localhost:8001/redoc>
  > - openapi <http://localhost:8001/docs>

- `nano`(占领 `8001` 端口)【`可以更改`】

- `nano` [`Nano` `mqtt` 消息说明](./nano/nano/README.md) (`Backend`需要关心)
  
  ```shell
  # 就是特地占位 强调一下  后端在开发 调度系统 和 路网编辑系统的时候要 关注这个
  # 就是特地占位 强调一下  后端在开发 调度系统 和 路网编辑系统的时候要 关注这个
  去打开一下 👆 README 连接
  ```

- `nano.yaml`配置文件说明
  
  ```yaml
  # 应用名称
  name: nano

  # 版本
  version: 1.0.0

  # FastAPI端口
  port: 8011

  # 设备编号
  device_no: RB0001

  # 环境, 默认开发环境（关闭 redoc / swagger / debug）
  # production or development
  environment: development

  # 日志级别与地址
  # DEBUG 、INFO、 WARNING OR WARN 、ERROR
  log_level: INFO 
  sys_log_file: logs/nano_biz.log
  access_log_file: logs/nano_access.log

  # MQTT配置
  mqtt: 
    local:
      host: 127.0.0.1
      port: 1883
      username: ''
      password: ''
    cloud:
      host: ''
      port: ''
      username: ''
      password: ''

  ```

- 内置的测试指令
  
  ```shell
    curl --request POST \
    --url http://127.0.0.1:8001/api/v1/publish \
    --header 'content-type: application/json' \
    --data '{
        "topic":"/nano/cmd/RB0001/nano_task",
        "msg":"wocao404",
        "qos": 1,
        "client": "local"
    }'
  ```
