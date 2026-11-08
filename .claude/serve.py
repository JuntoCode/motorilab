# Dev server without browser caching, plus a dev-only POST /__save endpoint used by
# js/dev/capture.js to write screenshots (.claude/shots/) and card thumbnails (assets/thumbs/).
import http.server, functools, sys, os, base64, urllib.parse
ALLOWED = ('.claude/shots/', 'assets/thumbs/')
class Dev(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()
    def do_POST(self):
        u = urllib.parse.urlparse(self.path)
        q = urllib.parse.parse_qs(u.query)
        path = (q.get('path') or [''])[0].replace(chr(92), '/')
        if u.path != '/__save' or '..' in path or not path.startswith(ALLOWED):
            self.send_error(403); return
        data = self.rfile.read(int(self.headers.get('Content-Length', 0))).decode()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            f.write(base64.b64decode(data.split(',', 1)[1]))
        self.send_response(200); self.end_headers(); self.wfile.write(path.encode())
port = int(sys.argv[1]) if len(sys.argv) > 1 else 5173
http.server.ThreadingHTTPServer(('127.0.0.1', port), functools.partial(Dev, directory='.')).serve_forever()
