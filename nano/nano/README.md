
# NANO 的 相关 Topic 定义

## 本机软件监听的 MQTT TOPIC 格式, 并转换成 ROS 话题

本机监听的是这个机器编号下的通配指令 `/nano/cmd/{device_no}/#`, 其中`{device_no}` 是机器人唯一编号

 也就是说,当有多个指令的时候，真实上位机发送的 MQTT TOPIC 格式是 `/nano/cmd/{device_no}/{cmd_name}`, 其中 `{cmd_name}`是指令名称

`{cmd_name}` 映射到本机本体下可以取值范围为当前本机的已经存在的`ROS`的`TOPIC名称`

例子:

> - 本机监听 `mqtt topic`: `/nano/cmd/R0001/#`
> - 上位机发布 `mqtt topic`: `/nano/cmd/R0001/ping_pong`
> - 本机监听到 该消息后，会提取 `/ping_pong` 作为发布 `ROS话题` 消息的`Topic`名称

## 本机软件监听的 ROS 话题, 接收到的消息数据会转录为 MQTT TOPIC 消息进行发布

本机监听 ROS 话题, `/{ros_topic_name}`, 当接收到数据后，会将数据转录, 区分类型, 根据 ROS 的话题名称转换成 MQTT 映射的 TOPIC 名称, 然后将这份数据发送给  `/nano/up/{device_no}/{topic}`

例子:

> - 本机监听 `ros 话题`: `/ping_pong`, 并获取消息
> - 根据 `/ping_pong` 找寻到映射 `/mqtt_ping_pong`, 后面上报数据到 `mqtt topic`: `/nano/up/R0001/mqtt_ping_pong`
> - 上位机监听 `mqtt topic`: `/nano/up/R0001/mqtt_ping_pong`
