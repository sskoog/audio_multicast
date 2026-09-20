# Extracting mp3 audio from YouTube

* Use `yt-dlp` tool.
* Requires `ffmpeg` for post-processing.
* Use flag `--audio-quality` to control output mp3 compression. 


## Recommended usage prompt
Using for 192 kbps CBR mp3 (good quality audio apt for streaming):
```bash
yt-dlp -x --audio-format mp3 --audio-quality 192K --embed-metadata --embed-thumbnail <youtube_url>
```

Install using winget:
```bash
winget install ffmpeg
winget install yt-dlp
```

## VBR vs CBR
Using variable bit rate (VBR) for mp3 generally results in better audio quality per file size for dynamic audio (music). However, VBR-files must be read in big chunks (sometimes the entire file) before decoding. This can be a problem for embedded devices, like decoding VBR mp3 on an ESP32. If you are streaming mp3 from an ESP32, it is recommended to use CBR, which can be decoded in small chunks.

Quick guide on mp3 quality (bit rate):
| Quality tier | Bit rate | --audio-quality |
|---|---|---|
| YouTube free | VBR ~140–185 kbps | 4 or 3 |
| YouTube free | CBR 192 kbps | 192K |
| YouTube Premium | VBR 220–260 kbps | 0 |
| YouTube Premium | CBR 256/320 kbps | 256K or 320K | 


## YouTube Audio quality
YouTube caps its standard streaming audio at a maximum bit rate of roughly 128 kbps AAC (format 140) or 160 kbps Opus (format 251). Your saved mp3 will not get better audio quality than this baseline.

### YouTube sample rate
YouTube’s primary guidelines recommend that creators upload music assets in 48 kHz, so almost all modern high-bitrate music streams are natively rendered at 48 kHz.

The default YouTube codec for modern **video** streams is Opus @ 48 kHz sample rate (format 251: webm).

Older legacy containers and specific YouTube Music standalone track distributions stream native 44.1 kHz AAC files (format 140: m4a).

## YouTube Premium?
You can get higher audio quality with a YouTube Premium subscription:

| Platform | Free Account Quality | Premium Account Quality | Codecs Used |
|----------|----------------------|-----------------------|-------------|
| YouTube Music | 128 kbps (Normal) | 256 kbps (High) | AAC & Opus |
| Main YouTube App | ~128 kbps standard | 256 kbps ("Enhanced Bitrate") | AAC & Opus |

However, higher bit rates are only available through YouTube's official apps.
Using yt-dlp will poll youtube's public-facing servers and only access the default audio quality (~128 kbps AAC or ~160 kbps Opus tracks).

## yt-dpl YouTube Premium hack
To download the actual high-quality tracks using your YouTube Premium subscription, you need to pass your web browser's authentication cookies directly into yt-dlp.

By default, yt-dlp has a built-in feature that can securely read cookies directly from your active browser profile on your PC, so you don't even need to manually export a file.

### Option 1: Share Cookies Directly from Your Browser (Easiest)
Make sure you are logged into your Premium account on your browser (e.g., Chrome, Firefox, Edge, or Brave), close the browser to release the database lock, and then add the '''--cookies-from-browser''' flag to your command: 
```yt-dlp --cookies-from-browser chrome -x --audio-format mp3 --audio-quality 256K --embed-metadata --embed-thumbnail "YOUTUBE_URL"```
Replace chrome with firefox, edge, brave, or safari depending on what you use.


### Option 2: Export a Cookies File Manually
If you use a private browser profile, extensions, or run your scripts on a machine where the browser isn't installed, you can export a cookies text file instead:
* Install a trusted browser extension like Get cookies.txt LOCALLY (available on Chrome/Edge store) or cookies.txt (on Firefox).
* Go to YouTube, make sure you are logged into your Premium account, click the extension icon, and select Export.
* Save the file as youtube-cookies.txt into your working folder (e.g. .\data\mp3 ).
* Run your command using the --cookies flag pointing to that file:
```yt-dlp --cookies youtube-cookies.txt -x --audio-format mp3 --audio-quality 256K --embed-metadata --embed-thumbnail "YOUTUBE_URL"```

### PO tokens
YouTube forces clients to generate a cryptographic attestation token (a PO Token) to prove the request is originating from a real app or browser session and not a script. Furthermore, YouTube now binds these tokens directly to the specific Video ID. This means old-school tricks like manually copy-pasting a token from a web browser window no longer work because the token becomes completely invalid the second you switch to a different song.

The official yt-dlp team features a plugin named bgutil-ytdlp-pot-provider. See more at `https://github.com/Brainicism/bgutil-ytdlp-pot-provider`.

### How to verify premium quality output
When you run the command with cookies enabled, keep an eye on the initial terminal output log lines. Look for the [info] line listing the format.
Instead of choosing the default 251 format (the standard free 160k Opus stream), an authenticated Premium session allows yt-dlp to request and download the enhanced high-bitrate audio stream containers directly from Google's servers.

## How to check available audio streams
Use this command to poll available streams before downloading:
```
yt-dlp -F "YOUTUBE_URL"
```
To get info only about the **audio** streams:
```
yt-dlp -F "YOUTUBE_URL" | findstr /C:"audio only"
```
And how to get info about **Premium** streams (if you did the cookie-export trick):
```
yt-dlp --cookies youtube-cookies.txt -F "YOUTUBE_URL"
```