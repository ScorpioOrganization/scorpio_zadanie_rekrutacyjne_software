# Projekt Scorpio - zadanie rekrutacyjne do działu Software
W celu realizacji zadania konieczne będzie zainstalowanie ROS2 w wersji Humble (zalecany system operacyjny to Ubuntu 22.04).
Na repozytorium znajduje się paczka ROS2 umożliwiająca połączenie z symulacją, oraz sama symulacja.
>**Uwaga!** Przed przystąpieniem do realizacji zadania przeczytaj **całe** README.
## Spis treści
- [Informacje ogólne](#informacje-ogólne)
- [Zadania do wykonania](#zadania-do-wykonania)
- [Specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania)
  - [Specyfikacja danych](#specyfikacja-danych)
  - [Uruchamianie symulatora](#uruchamianie-symulatora)
- [Wskazówki i przydatne linki](#wskazówki-i-przydatne-linki)
- [Przydatne ROSowe komendy CLI](#przydatne-rosowe-komendy-cli)

## Informacje ogólne
Zadanie wykorzystuje trójwymiarową symulację łazika marsjańskiego, w której możliwy jest ruch poszczególnymi silnikami kół, odczyt danych z sensorów oraz podgląd obrazu z kamery. Symulacja umożliwia połączenie z ROS2, co pozwala na wysyłanie oraz odczyt danych. Obraz z kamery jest widoczny w okienku po uruchomienu symulacji.

Łazik znajduje się w zamkniętej tubie o szerokości ~3.5m z płaską nawierzchnią. Wewnątrz tuby znajduje się zdjęcie z jaskrawą, czerwoną ramką.
Zdjęcie jest losowo umieszczane w tubie przy uruchomieniu symulacji.

Całość zadania należy wykonać używając frameworku Robot Operating System 2, a kod może być napisany w C++ lub Python.

## Zadania do wykonania 
W tej części znajdziesz ogólny opis zadań, szczegółowy opis wraz ze specyfikacją techniczną znajdziesz w sekcji [specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania).

Pamiętaj, że zadanie służy sprawdzeniu wielu umiejętności - nie tylko programowania i znajomości algorytmów -  więc nawet w przypadku zrealizowania tylko części z poniższych punktów, zachęcamy do przesłania rozwiązania. Postępy w zadaniu powinny być udokumentowane w repozytorium na githubie (po każdym etapie zadania powinien zostać stworzony nowy commit).

1. **Instalacja ROS2 i budowanie paczki do połączenia z symulacją:**
- W repozytorium została przygotowana paczka ROS2 zawierająca narzędzie umożliwiąjące podłączenie symulacji z ROSem. Repozytorium należy sklonować i zbudować paczkę w ROS2. Szczegóły działania paczki są opisane w sekcji [specyfikacja techniczna zadania](#specyfikacja-techniczna-zadania).

> **Wskazówka!** Dobrym rozwiązaniem jest "fork" tego repozytorium

2. **Stworzenie node'a ROSowego do sterowania łazikiem:**
- Node na podstawie danych wejściowych z klawiatury (WASD/strzałki) generuje odpowiednie prędkości dla każdego z silników.
- Node publikuje dane na topic'i `/wheel_XX/set_target_velocity`, gdzie XX to oznaczenie koła (`LF`, `RF`, `LR`, `RR`), a dane to prędkość obrotowa koła.
- Node umożliwia jazdę przód/tył, skręcanie w miejscu w lewo/prawo oraz jazdę po łuku.

3. **Stworzenie node'a do autonomicznego przejazdu przez tubę:**
- Node subskrybuje topic'i `/sensor/left` i `/sensor/right` zawierające odczyty z czujników odległościowych znajdujących się po lewej i prawej stronie łazika.
- Node na podstawie odczytów z sensorów steruje łazikiem tak, aby łazik autonomicznie mógł przejechać przez całą tubę.
- Autonomiczny przejazd powinno dać się uruchamić/zatrzymać za pomocą serwisów ROSowych `/enable_autonomy` i `/disable_autonomy`.

> **Wskazówka!** Pamiętaj o regularnym commitowaniu zmian

4. **Stworzenie node'a do detekcji zdjęcia w tubie:**
- W tubie, w losowo wybranym miejscu znajduje się nieznane zdjęcie z jaskrawą, czerwoną ramką.
- Node subkrybuje obraz z kamery umieszczonej na maszcie łazika z topicu `/camera/image`.
- Do tego zadania nie wolno używać obrazu w oknie symulacji.
- Za pomocą wybranej biblioteki do przetwarzania obrazu należy zaimplementować algorytm detekcji kolorowej ramki.
- Obraz należy wykryć w trakcie autonomicznego przejazdu, a po jego wykryciu node powinien zapisać na dysku klatkę obrazu z kamery z wykrytym zdjęciem.
- Na obrazie zapisanym na dysku powinno być wyraźnie widać zdjęcie w ramce.

> **Uwaga!** Zdjęcia w ramce mogą być różne, ale zawsze będą miały czerwoną ramkę, dlatego detekcja powinna być oparta na samej ramce.

## Specyfikacja techniczna zadania
> **Uwaga!** Nie modyfikuj plików utworzonych przez nas znajdujących się w paczce ROS.

**Nazwa paczki ROS** - `simulation_endpoint`  

- **simulation_endpoint** - przygotowany przez nas package służący do połączenia symulacji z ROSem za pomocą protokołu TCP.
- **rover_simulation** - stworzona przez nas symulacja łazika marsjańskiego oparta o silnik Unity. Sama w sobie działa poza ROSem, ale pozwala na połączenie z ROSem za pomocą paczki `simulation_endpoint`, dzięki czemu możliwe jest sterowanie silnikami i odczyt danych z sensorów i kamery.

### Specyfikacja danych
#### 1. Topic'i `/wheel_XX/set_target_velocity`

Topic'i te są odpowiedzialne za ustawianie prędości docelowej poszczególnych silników kół, gdzie:
- `/wheel_lf/set_target_velocity` - lewe przednie koło
- `/wheel_lr/set_target_velocity` - lewe tylne koło
- `/wheel_rf/set_target_velocity` - prawe przednie koło
- `/wheel_rr/set_target_velocity` - prawe tylne koło

Mają one być subskrybowane przez node'y obsługujące jazdę łazikiem. Ich typ wiadomości to Float32 (std_msgs/msg/Float32).
Wartością wiadomości jest prędkość obrotowa koła w radianach na sekundę. Wartość dodatnia oznacza ruch do przodu względem łazika, a ujemna do tyłu.
Jest to prędkość, którą kontroler PID silnika w symulacji będzie starał się osiągnąć.

> **Uwaga!** Kontrolery PID w symulacji mają ograniczony maksymalny moment obrotowy, więc w przypadku zbyt dużej prędkości, silnik nie będzie w stanie jej osiągnąć.

#### 2. Topic `/sensor/left` i `/sensor/right`

Topic'i te są odpowiedzialne za przesyłanie danych z sensorów odległościowych znajdujących się po lewej i prawej stronie łazika umieszczonych pod kątem.
Mają być subskrybowane przez node realizujący zadanie 3. Ich typ wiadomości to Float32.
Wartością wiadomości jest odległość od najbliższej przeszkody w metrach.
Czujniki te mają maksymalny zasięg wynoszący 3 metry.
Jeżeli zwracaną wartością jest zasięg maksymalny, to znaczy, że nie wykryto żadnej przeszkody.

Położenie sensorów na łaziku i ich zasięg przedstawia poniższy rysunek

![Lokalizacja sensorów](img/zad3_sensor.png)

#### 3. Serwis `/enable_autonomy` i `/disable_autonomy`
   
Serwisy te są odpowiedzialne za włączenie i wyłączenie autonomicznego przejazdu łazika. Ich typ wiadomości to Empty (std_msgs/msg/Empty).
Powinny być obsługiwane przez node z zadania 3.
Po wywołaniu serwisu `/enable_autonomy` node powinien rozpocząć autonomiczny przejazd, a po wywołaniu serwisu `/disable_autonomy` node powinien zatrzymać łazik.

#### 4. Topic `/camera/image`

Topic ten jest odpowiedzialny za przesyłanie obrazu z kamery umieszczonej na maszcie łazika.
Musi być subskrybowany przez node realizujący zadanie 4. Jego typ wiadomości to Image (sensor_msgs/msg/Image). Właściwości obrazu:
- Format: RGBA8
- Rozdzielczość: 720x480
- Częstotliwość: 30Hz

### Uruchamianie symulatora
1. Zbuduj paczkę ROS2 `simulation_endpoint`.
2. Pobierz najnowszy [Release](https://github.com/ScorpioOrganization/scorpio_zadanie_rekrutacyjne_software/releases) symulacji
3. Rozpakuj pobrany plik
4. Uruchom `simulation_endpoint` za pomocą komendy w terminalu:
```bash
ros2 launch simulation_endpoint endpoint.launch.py
```
5. Uruchom symulację za pomocą pliku wykonywalnego `rover_simulation.x86_64` (lub `rover_simulation.exe`) znajdującego się w rozpakowanym folderze symulacji. 

> **Uwaga!** W przypadku używania systemu WSL, symulacja musi być uruchamiana w systemie Windows.

## Wskazówki i przydatne linki
-	Zachęcamy do zapoznania się z poradnikiem przedstawiającym podstawy pracy w ROS2: https://www.youtube.com/watch?v=Gg25GfA456o
-	Oficjalny tutorial ROS2 znajdziesz pod linkiem: https://docs.ros.org/en/humble/index.html
-	Do instalacji ROS można wykorzystać instrukcję (należy wybrać wersję ros-humble-desktop): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html 
- Do ręcznego wysłania danych na topic w ROS możesz użyć komendy `ros2 topic pub <nazwa_topicu> <typ_danych> <dane>` (po wpisaniu nazwy topicu dobrze jest od razu użyć TAB aby powłoka pomogła w wpisywaniu i zajęła się typem danych i formatem). Możesz to wykorzystać do weryfikacji działania topic'ów z symulacji oraz swoich node'ów.
- Możesz użyć komendy `ros2 topic echo <nazwa_topicu>` aby wyświetlić dane wysyłane na określony topic.
- Zadanie rekrutacyjne można oddać niepełne.
- Rozwiązane zadanie należy umieścić w **publicznym** repozytorium (np. GitHub) i przesłać linka do tego repozytorium na mail projekt@scorpio.pwr.edu.pl. Ewentualne pytania lub wątpliwości co do treści zadania można kierować na tego samego maila. Zadania przyjmujemy do XX.XX.2025 do końca dnia.

## Przydatne ROSowe komendy CLI: 
- ros2 topic list - zwraca liste wszystkich dostępnych topiców
- ros2 topic echo <nazwa_topicu> - zwraca dane publikowane na topicu
- ros2 topic info <nazwa_topicu> - zwraca informacje o topicu
- ros2 topic pub <nazwa_topicu> <typ_message'a> <zawartość_message'a> - publikuje dane na topic
- ros2 topic hz <nazwa_topicu> - zwraca częstotliwość publikacji na topic
- ros2 service list - zwraca liste wszystkich dostępnych serwisów
- ros2 service type <nazwa_serwisu> - zwraca informacje o serwisie
- ros2 service call <nazwa_serwisu> <typ_serwisu> <zawartość_requestu> - wywołuje serwis i zwraca odpowiedź wywołania
- ros2 node info <nazwa node'a> - zwraca informacje o node'dzie


**Jeżeli będziesz miał jakiekolwiek wątpliwości i problemy z zadaniem śmiało skontaktuj się z nami na maila projekt@scorpio.pwr.edu.pl! Powodzenia :)**
