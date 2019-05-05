# Unique identifier for lane line
uint8 handle
# Timestamp when lane line classification took place
time stamp
# Line ID for lane line (left: positive, right: negative)
int8 id
# Line type indicating color, solid, dashed, etc...
uint8 type
# Polynomial
float32[] coefficients