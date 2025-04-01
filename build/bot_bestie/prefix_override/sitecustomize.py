import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rex/colcon_ws/src/r2auto_nav_CDE2310/install/bot_bestie'
