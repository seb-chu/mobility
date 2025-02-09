import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sebastianchu/code/student-orgs/karura/mobility_ws/install/mobility_description'
