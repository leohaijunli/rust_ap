#!/usr/bin/env python3
"""Independently parse raw source instances and selected calibrated mag data."""
import argparse
import json
import math
import subprocess
from pathlib import Path
from pyulog import ULog
from check_ulog import px4_fields


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rust",type=Path,required=True)
    parser.add_argument("--output",type=Path,default=Path("target/magnetometer-selection"))
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    path=args.output/"selection.ulg"
    subprocess.run([str(args.rust.resolve()),str(path.resolve())],check=True)
    log=ULog(str(path))
    assert not log.file_corruption
    assert {(d.name,d.multi_id) for d in log.data_list} == {("sensor_mag",0),("sensor_mag",1),("vehicle_magnetometer",0)}
    for topic,filename in [("sensor_mag","SensorMag.msg"),("vehicle_magnetometer","VehicleMagnetometer.msg")]:
        assert log.message_formats[topic].fields == px4_fields(filename)
    for index,id in [(0,0x431021),(1,0x431009)]:
        data=log.get_dataset("sensor_mag",index).data
        assert data["device_id"].tolist()==[id]
        assert math.isclose(float(data["x"][0]),0.22,rel_tol=1e-6)
        assert math.isnan(float(data["temperature"][0]))
    selected=log.get_dataset("vehicle_magnetometer").data
    assert selected["device_id"].tolist()==[0x431021,0x431009,0x431009,0x431021]
    assert selected["calibration_count"].tolist()==[0,0,1,0]
    assert selected["timestamp_sample"].tolist()==[1_000_000,1_010_000,1_010_000,1_000_000]
    expected=[[0.22,0.07,0.43],[-0.06,0.20,0.40],[-0.06,0.19,0.40],[0.22,0.07,0.43]]
    for row,values in enumerate(expected):
        for axis,value in enumerate(values):
            assert math.isclose(float(selected[f"magnetometer_ga[{axis}]"][row]),value,rel_tol=1e-6,abs_tol=1e-7)
    result={"synthetic":True,"raw_instances":2,"selected_records":4,"calibration_change_and_failover":True,"file_corruption":False,"passed":True}
    (args.output/"report.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    main()
