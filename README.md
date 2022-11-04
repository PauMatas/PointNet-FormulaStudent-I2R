# 3d PointCloud Dataset

# Data Base

## Initialization

Execute the `PointCloudDB.py` file to locally create the database. This file will be located in the directory `data`.

```shell
$ python3 scripts/PointCloudDB.py
```

After doing that you can import the module in any python file and work with it:

```Python
from PointCloudDB import *

pc_table = PointCloudTable()
pc_table.insert_data(
    x=1.3524787425994873,
    y=-5.194655895233154,
    z=0.18726466596126556,
    datetime=datetime.fromtimestamp(1644842548.0269108)
)
print(pc_table.read_rows())
```

# Subscribers

## Point cloud subscriber

Fields: 

| name | offset | datatype | count |
| --- | --- | --- | --- |
| x | 0 | 7 | 1 |
| y | 4 | 7 | 1 |
| z | 8 | 7 | 1 |
| r | 18 | 7 | 1 |
| g | 17 | 7 | 1 |
| b | 16 | 7 | 1 |
| intensity | 20 | 7 | 1 |
| range | 24 | 7 | 1 |
| timestamp | 32 | 8 | 1 |
| ring | 40 | 4 | 1 |
