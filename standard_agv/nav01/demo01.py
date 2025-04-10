#!/usr/bin/env python
# -*- coding:utf-8 -*-


import threading
from sr_modbus_sdk import *
from fastapi import FastAPI, Request
import uvicorn


app_nav = FastAPI()

@app_nav.post("/labTour")
async def labTour(request : Request):

    form_data = await request.form()
    start = form_data.get("start")
    end = form_data.get("end")
    mb_server = SRModbusSdk()
    mb_server.connect_tcp('192.168.71.50')
    for i in range(start, end+1):
        mb_server.move_to_station_no(i, 1)
        mb_server.wait_movement_task_finish(1)  


if __name__ == "__main__":
    
    def run_nav_server():
        uvicorn.run(app_nav, host="0.0.0.0", port=8080)

    nav_thread = threading.Thread(target=run_nav_server)
    nav_thread.start()
