#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
@Author  : pinkhello
@Software: 代理ROS2数据、指令到MQTT应用
"""
import os
import requests

from typing import Optional

from ..caches.global_cache import nano_setting_cache
from ..logs import sys_log
from ..utils import load_params_yaml


def download_to_save(url: str, version: Optional[str] = None):
    """下载"""
    dir_path = nano_setting_cache['obj'].road_path
    os.makedirs(dir_path, exist_ok=True)
    v_file_path = os.path.join(dir_path, 'road.version')

    if os.path.exists(v_file_path):
        with open(v_file_path, 'r') as f:
            _version = f.read()
            if _version and version and str(version) == str(_version):
                return

    try:
        response = requests.get(url=url, timeout=5, verify=False)
        filename = 'road.json'
        if 'content-disposition' in response.headers:
            content_disposition = response.headers['content-disposition']
            filename_start_index = content_disposition.find('filename=')
            if filename_start_index != -1:
                filename = content_disposition[filename_start_index + len('filename='):]
                filename = filename.strip("\"'")

        file_path = os.path.join(dir_path, f"{filename}.bak")

        with open(file_path, 'wb') as f:
            f.write(response.content)
        os.rename(file_path, os.path.join(dir_path, filename))

        with open(v_file_path, 'wb') as f:
            f.write(str(version).encode('utf-8'))  # type: ignore

        sys_log.info(f"下载路网成功 {file_path} 版本{version}")
    except requests.exceptions.Timeout:
        sys_log.error(f"下载路网超时 {url}")
    except requests.exceptions.RequestException as e:
        sys_log.error(f"下载路网错误 {url}", e)


def read_to_upload(url: str, version: Optional[str] = None):
    dir_path = nano_setting_cache['obj'].maps_path
    os.makedirs(dir_path, exist_ok=True)
    file_names = []
    for file_name in os.listdir(dir_path):
        file_path = os.path.join(dir_path, file_name)
        if os.path.isfile(file_path):
            if file_path.endswith('yaml'):
                metadata = load_params_yaml(file_path)
                _version = metadata['version'] if 'version' in metadata else None
                if _version and version and str(version) == str(_version):
                    sys_log.info(f"版本一致, 放弃上传地图 {version}")
                    return
            file_names.append(file_name)

    files = []
    for file_name in file_names:
        files.append(('files', (file_name, open(os.path.join(dir_path, file_name), 'rb'), 'application/octet-stream')))

    if len(files) > 0:
        # 查看是不是 yaml，读取版本进行处理   
        try:
            response = requests.post(url=url, files=files, timeout=5, verify=False)  # type: ignore
            if response.status_code == 200:
                sys_log.info(f"上传地图文件成功 {file_names} {version}")
            else:
                sys_log.error(f"上传地图文件失败 {file_names}")
        except requests.exceptions.Timeout:
            sys_log.error(f"上传地图超时 {file_names}")
        except requests.exceptions.RequestException as e:
            sys_log.error(f"上传地图错误 {file_names}", e)
    else:
        sys_log.warn(f"本地无地图文件")
