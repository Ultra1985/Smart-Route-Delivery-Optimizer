from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
import json, subprocess, os
ROOT=os.path.dirname(os.path.abspath(__file__)); os.chdir(ROOT)
class Handler(SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path!='/api/optimize': self.send_error(404); return
        try:
            L=int(self.headers.get('Content-Length','0')); d=json.loads(self.rfile.read(L)); pts=d['points']
            lines=[f"{len(pts)} {int(d.get('vehicles',2))} {float(d.get('capacity',15))} {d.get('traffic','medium')}"]
            lines += [f"{p['name']}|{float(p['x'])}|{float(p['y'])}|{float(p.get('demand',1))}|{int(p.get('priority',1))}" for p in pts]
            blocked=d.get('blocked',[]); lines.append(str(len(blocked))); lines += [f"{int(a)} {int(b)}" for a,b in blocked]
            proc=subprocess.run([os.path.join(ROOT,'optimizer')],input='\n'.join(lines)+'\n',text=True,capture_output=True,timeout=10)
            if proc.returncode: raise RuntimeError(proc.stderr or 'optimizer failed')
            out=proc.stdout.encode(); self.send_response(200); self.send_header('Content-Type','application/json'); self.send_header('Content-Length',str(len(out))); self.end_headers(); self.wfile.write(out)
        except Exception as e:
            out=json.dumps({'error':str(e)}).encode(); self.send_response(400); self.send_header('Content-Type','application/json'); self.send_header('Content-Length',str(len(out))); self.end_headers(); self.wfile.write(out)
if __name__=='__main__': print('Smart Route Optimizer 2.0 → http://localhost:8080'); ThreadingHTTPServer(('127.0.0.1',8080),Handler).serve_forever()
