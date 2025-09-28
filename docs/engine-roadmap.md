# Engine Core Roadmap

Bu doküman, uçuş simülasyonu odaklı mevcut projeyi gerçek zamanlı bir oyun motoruna dönüştürürken izlenecek çekirdek geliştirme adımlarını açıklar. Amaç, oyun/simülasyon spesifik modülleri motor altyapısından ayırmak ve yeniden kullanılabilir bir temel kurmaktır.

## Kısa Vadeli Hedefler (Sprint 0-1)
- **Motor Döngüsü ve Zamanlama**
  - Alt sistemler için yaşam döngüsü (`initialize → start → update → fixedUpdate → shutdown`) tanımla.
  - Sabit zaman adımı (fixed timestep) ve değişken kare süresi (delta time) yönetimi için merkezi bir saat (`EngineClock`) ekle.
- **Alt Sistem Yönetimi**
  - Core seviyesinde alt sistem arabirimi (render, physics, input vb.) için kayıt/öncelik mekanizması kur.
  - Alt sistemler arası veri paylaşımı için hafif bir bağlam (`EngineContext`) oluştur.
- **Temel Diagnostik**
  - Kare hızı, toplam süre, alt sistem durumu gibi metrikleri izleyen `EngineTime` ve loglama altyapısı hazırla.

## Orta Vadeli Hedefler (Sprint 2-4)
- **Olay Sistemi ve Mesaj Kuyruğu**
  - Tek yönlü (publish/subscribe) ve isteğe bağlı senkron işleme destekleyen bir olay otobüsü ekle.
  - UI, input ve oyun mantığı arasındaki bağları gevşet.
- **Kaynak Yönetimi**
  - Tekillik (singleton) yerine bağımlılık enjeksiyonuna uygun, referans sayımlı kaynak havuzu tasarla.
  - Asenkron yükleme için görev kuyruğu hazırlıklarını yap.
- **Zamanlanmış Görevler / Job Sistemi**
  - Küçük multi-threaded iş parçaları için iş kuyruğu taslağı.
  - Sabit güncelleme dışındaki ağır hesaplamaları arka plana taşıma imkanı.

## Uzun Vadeli Hedefler (Sprint 5+)
- **Sahne Grafı ve Entity-Component System (ECS)**
  - Hierarchical scene nodları ve bileşen tabanlı veri modeli.
  - Motor alt sistemlerinin ECS ile iletişim kurabileceği adapter katmanları.
- **Platform Soyutlamaları**
  - Giriş, pencere, dosya sistemi, ağ gibi hizmetleri platform bağımsız arayüzlerle kapsülleme.
- **Araçlar ve Editor Entegrasyonu**
  - Sahne düzenleyici, gerçek zamanlı parametre değişimi, durum kaydetme/geri yükleme.

## İlk Uygulanacak Modül
Bu sprintte hedef: **Motor Döngüsü + Alt Sistem Yönetimi** katmanını temel işlevleriyle ayağa kaldırmak. Böylece mevcut uçuş simülasyonu modülü, motor içinde bir alt sistem olarak çalıştırılabilecek ve diğer alt sistemler eklenmeye hazır hale gelecektir.

---
Bu plan, yeni gereksinimler ortaya çıktıkça iteratif olarak güncellenecektir. Her sprint sonunda dokümanda "tamamlandı" ve "gündem" bölümleri yenilenecektir.
