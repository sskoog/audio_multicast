"""Serve the VSAF Control page (index.html) on http://localhost and open it in the browser.

WebHID needs a secure context: http://localhost counts as one, a page opened from disk (file://) may not.
Usage: python serve.py [port]   (default 8000; Ctrl+C to stop)
"""
import functools
import http.server
import pathlib
import sys
import webbrowser


def main() -> int:
    """Start a local static web server for this folder and open the page. Returns an exit code."""
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    folder = pathlib.Path(__file__).resolve().parent
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(folder))
    with http.server.ThreadingHTTPServer(("127.0.0.1", port), handler) as httpd:
        url = f"http://localhost:{port}/index.html"
        print(f"VSAF Control: {url}  (Chrome or Edge; Ctrl+C to stop)")
        webbrowser.open(url)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
