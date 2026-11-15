# KMS Ream Stacker – AR animation

## Folder
| Path | What |
|---|---|
| `scripts/build_stacker.py` | Builds the parametric model and the 28 s animation, then exports GLB and USDZ |
| `scripts/render_preview.py` | Renders the preview MP4 (camera, lights and floor are added here only) |
| `output/stacker.glb` | Android / web AR file (one animation clip, `StackerCycle`) |
| `output/stacker.usdz` | iPhone / iPad AR Quick Look file |
| `output/stacker_preview.mp4` | 1280×720 preview video |
| `output/stacker.blend` | Blender source scene |
| `web/` | Ready-to-host AR web page (`index.html` + model files + poster) |

## Rebuild after changing parameters
Edit the parameters block at the top of `build_stacker.py` (ream size, step, belt width, timings), then run:
```
"C:\Program Files\Blender Foundation\Blender 5.3\blender.exe" --background --factory-startup --python scripts\build_stacker.py -- output
copy output\stacker.glb web\ & copy output\stacker.usdz web\
```

## Viewing without hosting
- **iPhone:** send `stacker.usdz` by AirDrop, WhatsApp, email or Files, then tap it. AR Quick Look opens and loops the animation.
- **Android:** open `stacker.glb` in a viewer app (e.g. Files by Google, or "3D Viewer"). For proper floor placement, use the web page.

## Hosting the web page (needed for the QR code)
AR on phones needs **HTTPS**. Upload everything in `web/` to any static host, for example:
- **GitHub Pages:** create a repo, upload the `web/` files, then go to Settings → Pages → deploy from the main branch.
- **Netlify Drop:** drag the `web/` folder onto app.netlify.com/drop.
- **Your company website:** any folder served over HTTPS. The server should send `model/vnd.usdz+zip` for `.usdz` and `model/gltf-binary` for `.glb`.

Open the page on a desktop and it shows a QR code for its own address. Print that QR code for brochures or stands.

## Device requirements
- iOS 13+ (AR Quick Look), on any iPhone/iPad with an A9 chip or later.
- Android 8+ with Google Play Services for AR (ARCore), installed automatically on supported phones.
