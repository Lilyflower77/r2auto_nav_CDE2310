import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rex/colcon_ws/src/bot_bestie/install/bot_bestie'
