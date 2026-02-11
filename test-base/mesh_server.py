import http.server

class CORSRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()

if __name__ == "__main__":
    print("Starting Mesh Server on port 8000 with CORS enabled...")
    http.server.test(CORSRequestHandler, port=8000)
