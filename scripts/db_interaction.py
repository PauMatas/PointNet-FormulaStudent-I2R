from datetime import datetime

import pointcloud_db as db

pctable = db.PoseTable()

pctable.insert_row(
    pos_x=1.3524787425994873,
    pos_y=-5.194655895233154,
    pos_z=0.18726466596126556,

    ori_x=1.3524787425994873,
    ori_y=-5.194655895233154,
    ori_z=0.18726466596126556,
    ori_w=0.18726466596126556,
    
    datetime=datetime.fromtimestamp(1644842548.0269108)
)

# pctable.insert_data(
#     x=2.3524787425994873,
#     y=-5.194655895233154,
#     z=0.18726466596126556,
#     datetime=datetime.fromtimestamp(1644842548.0269108)
# )

# pctable.insert_data(
#     x=3.3524787425994873,
#     y=-5.194655895233154,
#     z=0.18726466596126556,
#     datetime=datetime.fromtimestamp(1644842548.0269108)
# )

# pctable.insert_data(
#     x=4.3524787425994873,
#     y=-5.194655895233154,
#     z=0.18726466596126556,
#     datetime=datetime.fromtimestamp(1644842548.0269108)
# )
