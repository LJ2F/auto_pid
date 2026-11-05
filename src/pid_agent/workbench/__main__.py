import argparse
from .server import serve

parser = argparse.ArgumentParser(description="通用 PID 本地工作台")
parser.add_argument('--host', default='127.0.0.1')
parser.add_argument('--port', type=int, default=8766)
parser.add_argument('--results')
parser.add_argument('--allow-origin', action='append', default=[])
parser.add_argument('--open-browser', action='store_true')
args = parser.parse_args()
serve(args.host, args.port, args.results, args.allow_origin, args.open_browser)
