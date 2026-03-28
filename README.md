# Tucil2_13524144_13524147

Program ini mengubah file objek polymesh menjadi bentuk voxel dengan mengaplikasikan algoritma divide and conquer untuk membentuk octree yang membagi bounding box menjadi 8 bagian.

### Author/Colaborator
- Jonathan Harijadi (13524144)
- Muh.Hartawan Haidir (13524147)

### Requirement
- g++ 7 atau lebih baru (C++ 17 compatible compiler)

### Cara Mengkompilasi/Menjalankan Program
1. Clone repository ini
2. Buka vscode folder repository ini dalam wsl untuk windows
3. Masukkan perintah ini sambil mengubah <nama_Initial_Object> menjadi nama objek yg dipilih di folder Initial_Object seperti cow atau line:
``` make run FILE=test/Initial_Object/<nama_Initial_Object>.obj DEPTH=5 ```
4. Masukkan file hasil objek ke website pelihat objek 3d untuk melihat visual hasil objeknya seperti online 3d viewer