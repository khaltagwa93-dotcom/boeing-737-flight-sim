# Boeing 737 Realistic Flight Simulator

محاكاة قيادة طائرة ركاب Boeing 737-800 بفيزياء 6DOF حقيقية ورسومات Three.js.

## التشغيل المحلي
```bash
npx serve .
# أو
python3 -m http.server 8080
```

افتح http://localhost:8080

## التحكم
- **W/S** : Pitch
- **A/D** : Roll  
- **Q/E** : Yaw
- **Shift / Ctrl** : Throttle
- **Space** : تشغيل/إيقاف المحركات
- **C** : تبديل الكاميرا
- **R** : إعادة تعيين

## التقنيات
- Three.js r170 (PBR, Shadows, ACES Tone Mapping)
- نموذج ديناميكا طيران 6DOF مخصص (Lift/Drag/Moments/Stall)
- ISA Atmosphere model
