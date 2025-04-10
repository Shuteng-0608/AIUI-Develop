#!/usr/bin/env python
# -*- coding:utf-8 -*-

import threading
import logging
from sr_modbus_sdk import *
from fastapi import FastAPI, Request, HTTPException
import uvicorn
from typing import Optional
from pydantic import BaseModel

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app_nav = FastAPI(
    title="Lab Navigation API",
    description="API for controlling lab tour navigation system",
    version="1.0.0"
)

class LabTourRequest(BaseModel):
    start: int
    end: int

@app_nav.post("/labTour", summary="Start lab tour", response_description="Tour completion status")
async def labTour(request: Request):
    """Start a lab tour moving through stations from start to end."""
    try:
        form_data = await request.form()
        start = form_data.get("start")
        end = form_data.get("end")
        
        # Validate inputs
        if not start or not end:
            raise HTTPException(status_code=400, detail="Start and end parameters are required")
        
        try:
            start = int(start)
            end = int(end)
        except ValueError:
            raise HTTPException(status_code=400, detail="Start and end must be integers")
            
        if start <= 0 or end <= 0:
            raise HTTPException(status_code=400, detail="Start and end must be positive integers")
        if end < start:
            raise HTTPException(status_code=400, detail="End must be greater than or equal to start")

        logger.info(f"Starting lab tour from station {start} to {end}")
        
        # Connect to Modbus
        mb_server = SRModbusSdk()
        try:
            if not mb_server.connect_tcp('192.168.71.50'):
                raise HTTPException(status_code=500, detail="Failed to connect to Modbus server")
            
            # Move through stations
            for i in range(start, end + 1):
                logger.info(f"Moving to station {i}")
                mb_server.move_to_station_no(i, 1)
                mb_server.wait_movement_task_finish(1)
                
            return {"status": "completed", "message": f"Successfully toured stations {start} to {end}"}
            
        except Exception as e:
            logger.error(f"Modbus operation failed: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Navigation error: {str(e)}")
        finally:
            mb_server.disconnect()
            
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")

def run_nav_server():
    """Run the FastAPI server in a separate thread."""
    uvicorn.run(app_nav, host="0.0.0.0", port=8080)

if __name__ == "__main__":
    logger.info("Starting navigation server...")
    nav_thread = threading.Thread(target=run_nav_server, daemon=True)
    nav_thread.start()
    nav_thread.join()  # Wait for thread to complete (though it normally runs indefinitely)