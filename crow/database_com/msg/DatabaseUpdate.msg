# This expresses a database update (new objects added/pose modified)
Header header                  # timestamp in the header is the acquisition time and frame

int32[] id                  # which ids were updated
int32[] update_id      #for each cube info about type of update in database1 - new object, 2 - updated object
