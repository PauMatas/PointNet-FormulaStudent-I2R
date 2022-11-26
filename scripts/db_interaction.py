import pointcloud_db as db

bbs = db.get_accumulated_bounding_boxes(sample_size=50)

print('bbs: ', len(bbs))

for x in bbs:
    print('bb: ', len(x))