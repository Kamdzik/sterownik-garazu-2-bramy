// wersja 1.183  dodane 433Hmz
//
//////////////////////////////program do sterownika garażu, plik główny /////////////////////////////////////////////1
/// biblioteki
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <avr/io.h>
#define DECODE_NEC
#include <IRremote.h>

#include <EEPROM.h>
#include <RCSwitch.h>

///
// definicja pinów
///
////////////////////// BRAMA DOLNA GRAZOWA////////////////////////////
#define silnikdolnalewo 33    // PC5 // kierunek lewy silnik dolna brama
#define silnikdolnaprawo 32   // PC4 // kierunek prawo silnik dolna brama
#define pwmsilnikdol 5        // PE5 // pwm silnik dolna brama
#define wejsciefoto1dol 26    // PG0 // wejscie fotokmórki 1 wewnętrzna  brama dolna
#define amperilnikdolnab A0   // PF0 // wejscie czujnik pradu silnika bramy dolnej 1
#define przycbragardolna A4   // PF4 // wejscie przycisku wewnetrznego sterowanie brama dolna 1
#define wejsciehaLL11 30      // PC2 // wejscie halla górny czujnik brama dolna
#define wejsciehaLL12 31      // PC3 // wejscie halla dolny czujnik brama dolna
#define zasczujnikidolnbr 15  // PB7 // wyjscie zasilania czujników  bramy grarazowej dolnej  (hall foto)
#define switch_dolna_szyna 35 // PC6 // wejscie dolne na przyciki krancówki zabezpieczajace szyne

////////////////////// WSPULNE BRAMA GRAZOWA////////////////////////////
#define zastrafasilnkibr 43 // PA1// wyjscie zasilania transformatora silników bramy grarazowej
#define czujnikzasilnik 23  // PD5 // wejscie czujnika zasilania transformatora silników bramy grarazowej

////////////////////// BRAMA GÓRNA GRAZOWA///////////////////////////
#define silnikgornalewo 2     // PE2 // kierunek lewy silnik górna brama
#define silnikgornaprawo 3    // PE3 // kierunek prawo silnik górna brama
#define pwmsilnikgora 4       // PE4 // pwm silnik górna brama
#define wejsciefoto1gor 27    // PG1 // wejscie fotokmórki 2 wewnętrzna  brama górna
#define amperilnikgornab A1   // PF1 // wejscie czujnik pradu silnika bramy górnej 2
#define przycbragargorna A5   // PF5 // wejscie przycisku wewnetrznego sterowanie brama górnej 2
#define wejsciehaLL21 28      // PC0 // wejscie halla górny czujnik brama górna
#define wejsciehaLL22 29      // PC1 // wejscie halla dolny czujnik brama górna
#define zasczujnikigorabr 14  // PB6 // wyjscie zasilania czujników  bramy grarazowej gornej  (hall foto)
#define switch_gorna_szyna 34 // PC7 // wejscie gorne na przyciki krancówki zabezpieczajace szyne

#define CE 8   // PB0  // ce nrf24l01
#define CSN 12 // PB4  // csn nrf24l01

#define scl 18 // PD0 // SCL I2C
#define sda 19 // PD1 // SDA I2C

#define RX1_RS485 20   // PD2 // RX UART1 RS485
#define TX1_RS485 21   // PD3 // TX UART1 RS485
#define WYBOR_RS485 22 // PD4 // TRYB RS485

////////////////////// CZUJNIKI//////////////////////////////
#define zasilanieczujkaru 13 // PB5 // wyjscie zasilania czujnika ruchu zewnetrzny
#define wejczujnikruchu 25   // PD7 // wejscie czujnik ruchuu zewnętrzny
#define czujnikfotrezys A3   // PF3 // wejscie czujnik fotorezystora
#define czujrodzajzasil 24   // PD6 // wejscie czujnik zasilania akumulator/sieć (zwiera do masy gdy sieć)
#define buzzer 37            // PA6 // pin buzera

////////////////////// OSWIETLENIE ////////////////////////////
#define przycoswietlenie A6     // PF6 // wejscie prtzycisku oswietlenia zewnetrznego i wewnetrzego
#define lampa_stoj3_dom 44      // PA0 // wyjscie zasilania lampy stojaca "3"  dom    #
#define lampa_stoj1_garaz 42    // PA2  // wyjscie zasilania lampy stojaca "1"  garaż
#define lampa_stoj2_srodek 41   // PA3 // wyjscie zasilania lampy stojaca "2"  środek #
#define lampa_wisz_zew_garaz 40 // PA4 // wyjscie zasilania lampy wiszaca garaż   $
#define lampa_wisz_wew_garaz 39 // PA5 // wyjscie zasilania lampy wewnatrz garaż

//////////////////////DODATKOWE PINY////////////////////////////
#define podczerwien_ir A2 // PF2 // wejscie czujnika podczerwieni
#define inter_433hzm 6    // PE6 // wejscie pinu przerwania do odbiornika 433MHz

// #define wolny_INT6              6    // PE6 // pin wolny
// #define wolny_INT7              7    // PE7 // pin wolny
// #define wolny_prog/rx0          0    // PE0 // pin wolny
// #define wolny_prog/tx0          1    // PE1 // pin wolny
// #define wolny_tosc2             16   // PG3 // pin wolny
// #define wolny_tosc1             17   // PG4 // pin wolny
// #define wolny_npn               38   // PA7 // pin wolny

/// zmienne

////////////////////czasy błędu i animacji ekrnau 7 seg///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

byte cyfrybledow[] = {0b11111111, 0b10000001, 0b11100111, 0b10010010, 0b11000010, 0b11100100, 0b11001000, 0b10001000, 0b11100011, 0b10000000, 0b11000000, 0b10100000, 0b10001100, 0b10011110, 0b10000110, 0b10011000, 0b10111000, 0b00111000, 0b10100100, 0b00100100, 0b10110000, 0b00110000, 0b10011100, 0b00011100, 0b10000101, 0b10100001, 0b00000001, 0b10001110, 0b00001110, 0b01001000, 0b11001000, 0b10111101, 0b11100111, 0b10101010, 0b00101010, 0b11010100, 0b01010100, 0b00000000};
/////////////////////pustoooooooo/00000000000/1111111111//2222222222//3333333333/44444444444/5555555555/666666666666/777777777777/888888888888/99999999,AAAAAAAAAAAA,bbbbbbbbbbb,ccccccccccc, ddddddddddd,EEEEEEEEEE,FFFFFFFFFF//F.F.F.F.F.//HHHHHHHHHH//H.H.H.H.H.H//PPPPPPPPPPPP//P.P.P.P.P.//tttttttttt/t.t.t.t.t.t./odwUUUUUU /  U       /0.0.0.0.0.0. oooooooooo o.o.o.o.o.o/  .S.S.S.S.   SSSSSSSSS      "| "       "  |"    3pionowo    3pionowo..  2pion_zew  2pion_zew..  8.8.8.
//////                    0            1           2            3           4           5          6           7            8         9             10          11            12 ///      13         14       15          16          17          18           19           20           21       22          23           24           25        26          27            28          29        30            31          32         33          34          35         36         37

byte animacjalewy[] = {0b11011111, 0b10011111, 0b10111111, 0b10111110, 0b11111110, 0b11111100, 0b11111101, 0b11111001, 0b11111011, 0b11111111};          //      /|\       ///
byte animacjaprawy[] = {0b11011111, 0b11001111, 0b11101111, 0b11101110, 0b11111110, 0b11110110, 0b11110111, 0b11110011, 0b11111011, 0b11111111};         //       |        ///
byte animacjaobaprzeciwnie[] = {0b11011011, 0b10010011, 0b10110111, 0b10110110, 0b11111110, 0b11101100, 0b11101101, 0b11001001, 0b11011011, 0b11111111}; //       |        ///

//
unsigned long czas, czasanimacji, czasanimacji_pilot433, czasbledu = 0;
int a = 9, b = 0, aaa = 5;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// zmienne buzzera///////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte buzzerwlacz = 1;       // właczenie buzerra jesli 1 włączony jesli 0 wyłaczony!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte werpikacz1 = 0;        // zmienna bzera zeby tylko raz ustawiał wejscia krutkie
unsigned long czaspikaczu1; // czas który zerujac włączamy pikniecie krutkie
byte werpikacz2 = 0;        // zmienna bzera zeby tylko raz ustawiał wejscia długie
unsigned long czaspikaczu2; // czas który zerujac włączamy pikniecie długie
byte werpikacz3 = 0;        // zmienna bzera zeby tylko raz ustawiał wejscia szybkie alarm
unsigned long czaspikaczu3; // czas który zerujac włączamy pikniecie szybkie alarm

/// zmienne błędów bram garażowych
byte blad = 0, stopp = 1, aa = 0;
byte bladA; ////////////////////////błąd czujnika halla gorna brama    H.
byte bladB; ////////////////////////błąd czujnika halla  dolna  brama  H
byte bladC; ////////////////////////błąd fotokomórki górna  brama      F.
byte bladD; ////////////////////////błąd fotokomórki dolna  brama      F
byte bladE; ////////////////////////bład zaduzego poboru pradu silnik  gorna  brama      P.
byte bladF; ////////////////////////bład zaduzego poboru pradu silnik  dolna  brama      P
byte bladG; ////////////////////////bład przy  pojawieniu sie zasilania mimo braku załalczenia   U
byte bladH; ////////////////////////bład zadługiego otwierania bramy gornej  O.   26
byte bladI; ////////////////////////bład zadługiego zamykania bramy gornej   o.   28
byte bladJ; ////////////////////////bład zadługiego otwierania bramy dolnej  0    1
byte bladK; ////////////////////////bład zadługiego zamykania bramy dolnej   o    27
byte bladL; ////////////////////////bład niepojawienia sie napiecia silników mimo załączenia |^|   (odwrucone U) 25
byte bladM; ////////////////////////bład zaniskiego poboru silnik dolny    S     30
byte bladN; ////////////////////////bład zaniskiego poboru silnik gorny    S.    29

byte bladR;    ////////////////////////bład załączenia krancowki szyny przy otwieraniu Dolna brama // 3 pionowe     33
byte bladS;    ////////////////////////bład załączenia krancowki szyny przy otwieraniu Górna brama // 3 pionowe...  34
byte bladRR;   ///////////////////////bład załączenia krancowki szyny przy zamykaniu Dolna brama //  2 poziome     35
byte bladSS;   ///////////////////////bład załączenia krancowki szyny przy zamykaniu Górna brama //  2 poziome...  36
byte bladRR_R; /////////////////////blad jesli switch szyny sie nie odbl;okuje po zjechaniu lub selsi wystąpią dwa dłędy bladR ibladRR 9
byte bladSS_S; /////////////////////blad jesli switch szyny sie nie odbl;okuje po zjechaniu lub selsi wystąpią dwa dłędy bladS ibladSS 37
unsigned long czaswyswietlaniabledow = 10000;
//

/////////////TRYB USTAWIEN///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte aktybrama1 = 1;     ////brama garazowa dolna  jeśli 1 wł,, jesli 0 bram WYŁACZONA /////warunek która brama ma byc uzrywana!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte aktybrama2 = 1;     ////brama garazowa góna  jeśli 1 wł,, jesli 0 bram WYŁACZONA/////warunek która brama ma byc uzrywana!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte zamykaniewnocy = 0; // warunek jesli 1 zamyka wszystkie bramy gdy tylko czujnik wykryje noc  !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////zmienne ADC/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float adcprzycbrgorna, adcprzycbrdolna, adcprzycoswietlenie;
//

/////////////////////////////////ZMIENNE BRAMA DOLNA UCHYLNA //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int pomiar_adc_prad_dolny;
float prad_dolny_odczyt_raz;
byte bladpradpodjdolna11 = 0, bladpradpodjdolna1 = 0;
unsigned long czasprzyczamykaniedolnaanimacja;
byte animacjawolnedolna = 10;
unsigned long czasprzyczamykaniedolna, czasprzycotwieraniedolna;
byte otworzdolnazopuznienie = 0;
byte brama1, brama11;
byte przyciski_brama_dolna = 1; /// 1->  przyciski są aktywne brama dolna !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!o ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybzabezpamper1 = 0;      /// 1 powykryciu pradu wyłacza wsystkko 0 jesli wykryje prad odraca ruch /do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
int pwmwsilnkdol;
float pradsilnikbrdolna, pradsilnikbrdolnachwila; // przechowanie pomiaru amper silnik dolny
float maxpradsillnikbrdolnazam = 10.94;           // !!    // max Amper silnik zamykanie,,do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!
float maxpradsillnikbrdolnaotw = 9.5;             // ust    // max Amper silnik otwieranie ,,do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
float minpradsilnikdolny = 1.0;                   // minimalny prad jaki moze pobierać silnik dolny ponizej tej wartosci jest bład i wyłącza wszystko!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!

int liczpomar1;                       /// pomiary amper
unsigned long czaspomiarampdolna;     // czas co zwieksza  liczpomar1
unsigned long czaspwmplussilnikdolna; // czas do zwiekszania pwmwsilnkdol
byte trybfotodolna = 0;               /// tryb foto gornej jesli  1 wyłacza wszytko jesli 0 wraca z powrotem  ,,do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czasopuznwlczujdol1;
unsigned long czasopuznwlczujdol2 = 1000;             ////czas czujników na ururchomienie po włączeniu zasilania !!lub czas!! opużnienia wł silnika po zmianie ruchu ,,do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czaswysblfotodolny, czaswysblhalldolny; /// zmienne czasu wyswietlania bledu halla ifotokomórki
unsigned long czasmaxotwudul1, czasmaxzamudul1, czaswysbladNtwdul, czaswysbladzamdol;
unsigned long czasmaxotwudul2 = 32000; // ust /// czas max w którym brama dól ma sie otworzyc do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czasmaxzamudul2 = 26000; // ust /// czas max w którym brama dól ma sie zamknąć do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czasopuzniwylprzekdolotw, czasopuzniwylprzekdolzam;
unsigned long zmczasopuzniwylprzekdolotw = 1000; // czas opuźnienia wyłączenia przekaźników dolnych otwieranie do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long zmczasopuzniwylprzekdolzam = 1000; // czaso puźnienia wyłączenia przekaźników dolnych zamykanie do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte waropuzdolotw, waropuzdolzam;
byte trybopuzdolotwhall = 1;  /// tryb opuźniena dolna otwieranie jesli 1 otwieranie powolne z opuźnieniem jesli 0 otwieranie ostre HALL!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzdolzamhall = 1;  /// tryb opuźniena dolna zamykanie jesli 1 zamykanie powolne z opuźnieniem jesli 0 zamykanie ostre HALL!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzdolotwprad = 1;  /// tryb opuźniena dolna otwieranie jesli 1 otwieranie powolne z opuźnieniem jesli 0 otwieranie ostre PRAD!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzdolzamprad = 1;  /// tryb opuźniena dolna zamykanie jesli 1 zamykanie powolne z opuźnieniem jesli 0 zamykanie ostre PRAD!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzdolotwpilot = 1; /// tryb opuźniena dolna otwieranie jesli 1 STOP powolne z opuźnieniem jesli 0  STOP ostre  PRZYCISK PRZECIWNY NA  PILOCIE !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzdolzampilot = 1; /// tryb opuźniena dolna zamykanie jesli 1 STOP powolne z opuźnieniem jesli 0 STOP  ostre PRZYCISK PRZECIWNY NA  PILOCIE !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzdolzamfoto = 1;  /// tryb opuźniena dolna zamykanie jesli 1 STOP powolne z opuźnieniem jesli 0 STOP  ostre FOTOKOMÓRKA !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czaswysbladpraddul, czasminpradsilnikdolny, czasbladpraddolny;
unsigned long czaswykrniskpradusildul = 3000; // czas w którym ma sie zwiekszyc pobor pradu podczas załączenia inaczej blad silnik dolny !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte warprzycbrdul;

// czujki szyny
byte czujka_dolna_szyna; // zmienna czujników dolnej bramy szyny
unsigned long czas_wys_bl_sw_dolny, czas_wylna_srodku_dol;
byte drugieczujki_szyny_dolna = 1; /// 1 czujniki szyny włączone, 0 wyłączone!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!
byte tryb_szynadolna_1 = 1;        /// szyna dolnej bramy jesli switch rozwewrze i jest równe "1" to potrót jesli 0 stop !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte tryb_szynadolna_11 = 1;       /// jesli zero powolne, jesli 1 szybkie stawanie lub powrót zaleznie od tryb_szynadolna_1 !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte var_stanu_szyna_dolna = 0;    // zmienna pomocnicza do opuźnienia

byte czujnik_pradu_dolna_brama = 1; // 1 czujniki pradu właczone jesli 0 czujniki pradu wyłączone!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czas_zwloki_brdolna_otworz_prad_licz;
unsigned long czas_zwloki_brdolna_zamknij_prad_licz, czas_filtr_przyc_dolna2, czas_filtr_przyc_dolna1;
unsigned long czas_zwloki_brdolna_prad = 2100; // jesli przez ten czas prad silnika bedzie przekraczał maksymalna wartosc wtedy wywala bład przy otwieraniu brama dolna!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!
//
//

//
////zmienne bram wspulne///////////////////////////////////////////////////////////////////////////
byte zastrafabram, zastrafabram_gora, zastrafabram_dol;
byte warwylwszystbrakzas; /// warunek do wyłączenia wszystkiego TYLKO RAZ w petli jesli zasilanie jest załączone
unsigned long czaswlaczeniazass1, czaswysbladwlzasil;
unsigned long czaswlaczeniazasilania1 = 7000; /// czas w ktorym napiecie zasilania silników ma sie pojawic  !!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czaswysbladwylzasil, czaswylaczeniazass1;
unsigned long czaswylaczeniazasilania1 = 7500; /// czas w ktorym napiecie zasilania silników ma zniknąć!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czasprzycdlugie = 2000;          /// czas po jakim jest przycisniecie długie !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czsaopuszczeniezwloka = 3500;    /// czas co ile ma odliczac w dół przy zwlekajacym zamykaniu bram !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czas_pwm_wlacz = 0;              // w ms  /// czas co jaki ma sie zwiekszac pwm silników!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte rodzaj_zasilania;                         // zmienna przechowujaca rodzaj zasilania  akuulator/siec/////0--> siec // 1-->akumulator
byte zmien_czuj_zas;                           // /////0--> jest napiecie // 1-->nie ma napiecia na silnikach zmienna przechowujaca czujnik zasilania
byte wlacz_l_wew_polczasu = 1;                 /// 1-> lampa wewnętrzna włącza sie w połowie czasu otwierania w nocy, 0 -> nie włacza sie wogule!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte war_czas_autozamk = 0;
unsigned long czas_oudznia_auozamknij_licz;
unsigned long czas_oudznia_auozamknij = 1800000;   /// po 30 min /// po załączeniu nocy po tym czasie zamknią sie bramy !!!!!!!!!!!!!!!!!!!!!!!!!!!!! do ustwienia !!!!!!!!!!!!!!!!!
unsigned long czas_przelaczenia_zasilania = 5000;  // czas usuwajasy błąd pojawienia sie napiecia przy przełączaniu akumulator/siec
unsigned long czas_powrotu_po_bledzieszyny = 8000; // czas przez który wózek wraca po błędzie dojechania do krancówki szyny
unsigned long czas_co_ile_liczyprad = 0;
unsigned long czas_odlicz_czujka_silnikizasilania = 0; // czas liczący opuźnienie od braku zasilania
unsigned long czasnacisniencia_przyc = 70;             // ms jeśli przez ten czas bedzie naciśnienty przycisk od bram to wystartuje odpowiednia funkcja otwórz zamknij !!!!!!!!!!!!!!!!!!!!!!!!!!!!! do ustwienia !!!!!!!!!!!!!!!!!
//
//
/////////////////////////////////ZMIENNE BRAMA GÓRNA UCHYLNA //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int pomiar_adc_prad_gorny;
float prad_gorny_odczyt_raz;
byte bladpradpodjgorna11 = 0, bladpradpodjgorna1 = 0;
byte przyciski_brama_gorna = 1; /// 1->  przyciski są aktywne brama gorna !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!o ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czasprzyczamykaniegoraanimacja;
byte animacjawolnegora = 10;
unsigned long czasprzyczamykaniegora, czasprzycotwieraniegora;
byte otworzgorazopuznienie = 0;
byte brama2, brama22, zmienafotogorna;
byte trybzabezpamper2 = 0;                        // 1 powykryciu pradu wyłacza wsystkko 0 jesli wykryje prad odraca ruch /do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
int pwmwsilnkgor;                                 // zmienna przechowywujaca pwm silnik gorny
float pradsilnikbrgorna, pradsilnikbrgornachwila; // przechowanie pomiaru amper silnik gorny
float maxpradsillnikbrgornazam = 11.22;           // max Amper silnik zamykanie  ,,do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!nasz silnil
float maxpradsillnikbrgornaotw = 9.93;            // max Amper silnik otwieranie   ,,do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
float minpradsilnikgorny = 1.0;                   // minimalny prad jaki moze pobierać silnik gorny ponizej tej wartosci jest bład i wyłącza wszystko!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!

int liczpomar2;                      // pomiary amper
unsigned long czaspomiarampgorna;    // czas co zwieksza  liczpomar2
unsigned long czaspwmplussilnikgora; // czas co zwiekszania pwmwsilnkgor
byte trybfotogorna = 0;              // tryb foto gornej jesli  1 wyłacza wszytko jesli 0 wraca z powrotem  ,,do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czasopuznwlczujgora1;
unsigned long czasopuznwlczujgora2 = 1000;                                                ////czas czujników na ururchomienie po włączeniu zasilania !!lub podczas zmiany ruchu,,do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czaswysblfotogorny, czaswysblhallgorny;                                     /// zmienne czasu wyswietlania bledu fotokomórki i halla
unsigned long czasmaxotwugora1, czasmaxzamugora1, czaswysbladNtwgora, czaswysbladzamgora; // zmienne pomocnicze do max czasów otw i zambramy gornej
unsigned long czasmaxotwugora2 = 38000;                                                   // ust  /// czas max w którym brama gorna ma sie otworzyc do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czasmaxzamugora2 = 36000;                                                   // ust  /// czas max w którym brama gorna ma sie zamknąć do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czasopuzniwylprzekgorzam, czasopuzniwylprzekgorotw, czasbladpradgorny = 0;
unsigned long zmczasopuzniwylprzekgorotw = 1000; // czasopuźnienia wyłączenia przekaźników gornych otwieranie do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long zmczasopuzniwylprzekgorzam = 1000; // czasopuźnienia wyłączenia przekaźników gornych zamykanie do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte waropuzgorzam, waropuzgorotw;
byte trybopuzgorotwhall = 1;  /// tryb opuźniena gorna otwieranie jesli 1 otwieranie powolne z opuźnieniemjesli 0 otwieranie ostre HALL!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzgorzamhall = 1;  /// tryb opuźniena górna zamykanie jesli 1 zamykanie powolne z opuźnieniemjesli 0 zamykanie ostre HALL!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzgorotwprad = 1;  /// tryb opuźniena gorna otwieranie jesli 1 otwieranie powolne z opuźnieniemjesli 0 otwieranie ostre PRAD!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzgorzamprad = 1;  /// tryb opuźniena górna zamykanie jesli 1 zamykanie powolne z opuźnieniemjesli 0 zamykanie ostre PRAD!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzgorotwpilot = 1; /// tryb opuźniena gorna otwieranie jesli 1 STOP powolne z opuźnieniemjesli 0  STOP ostre PRZYCISK PRZECIWNY NA PILOCIE!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzgorzampilot = 1; /// tryb opuźniena górna zamykanie jesli 1 STOP powolne z opuźnieniem jesli 0 STOP ostre PRZYCISK PRZECIWNY NA PILOCIE! !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte trybopuzgorzamfoto = 1;  /// tryb opuźniena górna zamykanie jesli 1 STOP powolne z opuźnieniem jesli 0 STOP ostre FOTOKOMÓRKA !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czaswysbladpradgora, czasminpradsilnikgorny;
unsigned long czaswykrniskpradusilgora = 3000; // czas w którym ma sie zwiekszyc pobór pradu podczas załaczenia inaczej blad silnik gorny !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!

byte warprzycbrgora;

//
// czujki szyny
byte czujka_gorna_szyna; // zmienna czujników gorna bramy szyny
unsigned long czas_wys_bl_sw_gorny, czas_wylna_srodku_gor;
byte drugieczujki_szyny_gorna = 1;  // 1 czujniki szyny włączone, 0 wyłączone!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!
byte tryb_szynagorna_1 = 1;         /// szyna gornaej bramy jesli switch rozwewrze i jest równe 1 to potrót jesli 0 stop   !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte tryb_szynagorna_11 = 1;        // jesli zero powolne,, jesli 1 szybkie stawanie lub powrót zaleznie od tryb_szynagorna_1    !!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!
byte var_stanu_szyna_gorna = 0;     // zmienna pomocnicza do opuźnienia
byte czujnik_pradu_gorna_brama = 1; // 1 czujniki pradu właczone jesli 0 czujniki pradu wyłączone!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czas_zwloki_brgorna_otworz_prad_licz;
unsigned long czas_zwloki_brgorna_zamknij_prad_licz, czas_filtr_przyc_gorna2, czas_filtr_przyc_gorna1;
unsigned long czas_zwloki_brgorna_prad = 2200; // jesli przez ten czas prad silnika bedzie przekraczał maksymalna wartosc wtedy wywala bład przy otwieraniu brama gorna!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!
//
//
//
//
// czujnik zmierzchu // fotorozystor
unsigned long odczytfotorezys, adczmierzch;
int liczbapomiarfotorez;
unsigned long czaspomiarfotorezystor;
//
//////////////////////////////////////////////////////////////////////////////////////////
//

///////////////zmienne lamp///////////////////////////////////////////////
byte war_raz_czuj_ruchu_2 = 0, war_raz_czuj_ruchu_1 = 0;
byte czujnik_dom, czujnik_garaz;                                   // zmienna przechowująca stan czujnika nrf24l01
unsigned long wartoscdonocy = 870;                                 // wartosc 0-1023 po której jest noc !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia~~!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
byte noc = 0;                                                      // zmienna nocy
byte kroki_animacji = 0, kroki_animacji_1 = 0, wybor_animacji = 0; ////zmienna pomocnicza do liczenia animacji
unsigned long czas_animacjilamp;                                   /// czas pomocniczy do animacji bez czujników
unsigned long czas_animacjilamp_1;                                 /// czas pomocniczy do animacji z czujnikami

////nowe zmienne
byte wl_auto_wiszaca_noc = 1;                                             // zmienna  właczają lame wiczacą zewnetrzna wieczorem automatycznie// 1 włacza sie ze stojącymi/// 0 wył!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienai !!!!!!!!!!!!!!!!!!!!!
byte tryb_lamp_auto = 0;                                                  // tryb lamp wieczorem .gdzy jest noc //// 0 -> nic // 1 -> włączenie wieczorem ,wyłączenie rano./// 2 -> włączenie wieczorem ,wyłączenie po czasie np. po 3 godz.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienai !!!!!!!!!!!!!!!!!!!!!
byte war_wl_lamp_auto1 = 0, war_wl_lamp_auto2 = 0, war_wl_lamp_auto3 = 0; // warunek do pentli włączenia lamp
byte nr_animacji = 0;                                                     // numer animacji przy włączeniu i wyłaczeniuu
unsigned long czas_wl_lamp_wiecor_licz;                                   // czas ppomocniczy do wyłączenia lamp (włączenie wieczorem ,wyłączenie po czasie)
unsigned long czas_wl_lamp_wiecor = 10000;                                ///////1800000;//-->30min \\//czas po którym  wyłączeają sie  lampy w trybie  tryb_lamp_auto = 2;    (włączenie wieczorem ,wyłączenie po czasie) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawiena!!!!!!!!!!!!!!!!!!!!!
unsigned long czas_nastepnej_lampy_wl_wyl = 5000;                         // czas po jakim włacza sie nastepna lamp w animacji bbez czujników!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czas_opuz_czujniki_licz;                                    // pomocniczy czas opuźnienia czujników opświetlenia
unsigned long czas_opuz_czujniki = 100000;                                // czas opuźnienia czujników oświetlenia po jakim są aktywne !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czas_nastepnej_lampy_wl_wyl_czujniki = 4500;                // czas po jakim włacza/wylacza sie nastepna lamp w animacji z czujnikami!!!!!!!!!! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienia !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// zmienne wyjsciowe z sterowań
//
// zmienne sterowania z trybów noc dzien i po czasie BEZ CZUJNIKÓW//////////
byte lampa_st_dom_0_1 = 0;
byte lampa_st_sro_1_1 = 0;
byte lampa_st_gar_2_1 = 0;
byte lampa_wi_gar_3_1 = 0;
////////////////////////////////////////////////

byte lampa_wiw_gar_4_1 = 0; /// do bramy 1 dolna wewnatrz lampa
byte lampa_wiw_gar_4_2 = 0; /// do bramy 2 gorna wewnatrz lampa

// zmienne sterowania z trybów noc dzien i po czasie  Z CZUJNIKAMI//////////
byte lampa_st_dom_0_2 = 0;
byte lampa_st_sro_1_2 = 0;
byte lampa_st_gar_2_2 = 0;
byte lampa_wi_gar_3_2 = 0;

byte lampa_st_dom_0_3 = 0; // zmienna sterujaca lampą tylko przy domu od czujnka na  domie

byte war_przyjazd_droga = 0;         // zmienna pomocnicza do ustalenia kiedy lampy sa właczone przyjazd do domu
byte wyl_lamp_wisz_przyjazd = 0;     /////jesli 1 to lampa wiszaca wyłacza sie wczesniej niz lampy stojace //po  czasie //czas_wyl_lam_wisz|/   //// 0 -> wiszaca  wyłącza sie razem z stojącymi!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienai   !!!!!!!!!!!!!!!!!!!!!!!!!!!
byte war_wyl_lwisz_przyjsciedom = 0; // 1->> jesli czujnik garazu nie łapie a czujnik domu złapie i lampy sa włączone to wyłączy odrazu lame garażu wiszącą///jesli 0 to niewyłączy a wyłaczy sie po czasie sama z resztą lamp

unsigned long czas_wyl_lam_wisz_licz, czas_wyl_lam_sto_licz; /// czasy pomocnicze do wyłaczania lamp
unsigned long czas_wyl_lam_wisz = 7000;                      /// czas wyłaczenia lampy wiszącej wcześniej// jesli wyl_lamp_wisz_przyjazd==1 (jesli czujnik garazu nie łapie  po tym czasie wyłącza lampe wiszaca garaż wcześniej niz lampy stojące) !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawienai   !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czas_wyl_lam_sto = 20000;                      // czas po jakim wyłącza sie wszytkie lampy jesli czujnik garazu nie łapie  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustawiena !!!!!!!!!!!!!!!!!!!!!!!!!!

////////////////////////////////////////////////
// zmienne sterowania do domu

byte war_lampa_tylko_dom;
unsigned long czas_lampa_tylko_dom_licz;    // czas pomocniczy
unsigned long czas_lampa_tylko_dom = 20000; // czas jaki bedą swiecic lampy przy domu jesli nie złapie czujnik garażu
/// zmienne oswietlenie z przyciskami

byte przyciski_oswietlenie = 1; /// jesli 1 przyciski oswietlenia włączone jesli 0 wyłaczone ///////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! do ustawienia///////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
byte stan_lampa_stojaca = 3;    //// licznik sterowania lapami stojacymi 1 ->> właczone , 2 ->> wyłączone , 3 ->> automatycznie
byte warprzyc_stoj;             // zmienna pomocnicza do przycisku
byte stan_lampa_wewntrzna = 3;  /// licznik sterowania lapa wewnętrzna 1 ->> właczone , 2 ->> wyłączone , 3 ->> automatycznie
byte warprzyc_wewn;             // zmienna pomocnicza do przycisku

unsigned long czas_przyc_licz;  /// zmienna pomocnicza
unsigned long czas_przyc = 350; /////////////////////////////////czas odstępu pomiedzy kliknienciem (usuniecie zakłucen) do ustawienia 1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
unsigned long czas_wylacz_reczne_lampy_licz;
unsigned long czas_wylacz_reczne_lampy = 8000000; //-->30min//// czas po którym wszystkie ustawienia sterowania recznego lamp wrócą na automatyczne !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustwienia!!!!!!!!!!!!!!!!!!!!!!!!!!
                                                  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long czas_przelaczenia;
byte zmienna_przelaczenia = 1;

// zabezpieczenie czujki dom
unsigned long czas_zabezpiecz_czujka_dom_licz;
unsigned long czas_zabezpiecz_czujka_dom = 900000; ///->115min //// zabezpieczenie jesli szujka domu nie odpowie to po tym czasie lampy w domu sie wyłacza!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!do ustwienia!!!!!!!!!!!!
///
///

///
///  zmienne dotyczące odbiornika RCswitch

unsigned long dane_rcswitch;
byte tryb_ustawien_433 = 0; // tryb usatwien 433   1-pozwolenie z pierwszego  przycisku,,, 2-pozwolenie z drugiego przycisku ,, 3++ włączenie programowania kodów
// tryb_ustawien_433 == 3   otwórz brama dolna
// tryb_ustawien_433 == 4  zamknij brama dolna
// tryb_ustawien_433 == 5  otwórz brama górna
// tryb_ustawien_433 == 6  zamknij brama górna

unsigned long czas_przyc_dlugie_prog = 10000;     // 1min po tym czasie włącza sie usatwienie pilota
int nr_pilota = 0;                                /// 1,2,3,4,5,6//  0,16,32,64,128,256
byte nr_pilota_licz_plus = 0;                     // pomoc w liczeniu pilotów
int nr_dwa_eeprom_pilot_433, nr_eeprom_pilot_433; // zmienne do numerów kolumn i wierszy przy odczycie z pamięci
unsigned long tabRCswitch_bufor[6];               // bufor odczytanych komend przy zapisie
byte numer_tabRCswitch_bufor = 0;
unsigned long tabRCswitch[5][3]; /// PIERWSZALICZBA WIERSZE --  , DRUGA KOLUMNY ||   !!!OD ZERA!!
//      otwórz doł  ,,  zamnij dół ,, otwórz góra  ,,  zamnij góra
//          0               1              2               3
///  0    XXXXX           XXXXX          XXXXX           XXXXX
///  1    XXXXX           XXXXX          XXXXX           XXXXX
///  2    XXXXX           XXXXX          XXXXX           XXXXX
///  3    XXXXX           XXXXX          XXXXX           XXXXX
///  4    XXXXX           XXXXX          XXXXX           XXXXX
///  5    XXXXX           XXXXX          XXXXX           XXXXX

unsigned long czas_pilot1_klik, czas_pilot2_klik, czas_pilot3_klik, czas_pilot4_klik; // zmienne czasowe które sa do opóźnienia dwukliku
unsigned long czas_pilot_klik_dwuklik = 120;                                          //  czas po którym bedzie wykonany dwuklik przycisku 433Mhz
unsigned long czas_pilot_czas_ustawien = 100000;                                      // zas po którym ustawienia pilotów sie
unsigned long czas_pilot_czas_ustawien_licz;
byte animacja_pilot_k = 0;
/////

/////
RF24 radio(CE, CSN);

const uint64_t pipe = 0x1AE2ECF96LL; // kanał odbioru

struct receivedData
{
    byte numerpilota; ////////////////adres pilota do ustawienia
    bool ch1;
    bool ch2;
    bool ch3;
    bool ch4;
    bool ch5;
    bool ch6;
    bool ch7;
    bool ch8;
};
receivedData rxData;

RCSwitch mySwitch = RCSwitch(); ///

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// zmienne do uart 0 /pin programowania
byte monitoring = 0; //// jesli 1 to na uart 0 pojawiają sie dane ,, jesli 0 uart wył

///////////////

// wyjdscie zasilania czujnika ruchu
void czujka_ruchu_zasilanie(byte a)
{
    digitalWrite(zasilanieczujkaru, a);
}
///////////////////////////////przekaźniki bramy garazowej
void kieruneksilnikgor2(byte a)
{
    digitalWrite(silnikgornalewo, a); //////przekaznik lewo brama górna
}

void kieruneksilnikgor1(byte a)
{
    digitalWrite(silnikgornaprawo, a); //////przekaznik prawo brama górna
}

void kieruneksilnikdol2(byte a)
{
    digitalWrite(silnikdolnalewo, a); //////przekaznik lewo brama dolna
}

void kieruneksilnikdol1(byte a)
{
    digitalWrite(silnikdolnaprawo, a); //////przekaznik prawo brama dolna
}

void zasczujniki_dolnbr(byte a)
{
    digitalWrite(zasczujnikidolnbr, a); //////wyjscie zasilania czujników  bramy grarazowej dolnej  (hall foto)
}

void zasczujniki_gorabr(byte a)
{
    digitalWrite(zasczujnikigorabr, a); //////wyjscie zasilania czujników  bramy grarazowej gornej  (hall foto)
}

void pcf8574(byte a)
{
    Wire.beginTransmission(0x20);
    Wire.write(a);
    Wire.endTransmission();
}

void wyl_zas_gora()
{
    if (brama2 == 0 && brama22 == 0)
    {
        zastrafabram_gora = 0;
    }
}

void wyl_zas_dol()
{
    if (brama1 == 0 && brama11 == 0)
    {
        zastrafabram_dol = 0;
    }
}

void bramagornazamykanie()
{

    if (brama2 == 0)
    {
        if (brama22 == 0)
        {
            if (bladA == 0 && bladG == 0 && bladL == 0 && waropuzgorotw == 0 && waropuzgorzam == 0 && bladE == 0 && bladC == 0 && bladS == 0 && bladSS == 0 && bladSS_S == 0)
            {
                zastrafabram_gora = 1;
                brama22 = 1;
                zasczujniki_gorabr(1);
                czasopuznwlczujgora1 = czas;
                pwmwsilnkgor = 0;
                kieruneksilnikgor2(1);
                czasmaxzamugora1 = czas;
                czasminpradsilnikgorny = czas;
            }
        }
    }
    else
    {
        zasczujniki_gorabr(0);
        pwmwsilnkgor = 0;
        brama2 = 0;
        if (trybopuzgorotwpilot == 0) /// ostrree
        {
            wyl_zas_gora();
            kieruneksilnikgor1(0); //////przekaznik prawo brama górna
        }
        else /// powolne
        {
            waropuzgorotw = 1;
            czasopuzniwylprzekgorotw = czas;
        }
    }
}

void bramagornaotwieranie()
{
    if (brama22 == 0)
    {
        if (brama2 == 0)
        {
            if (bladA == 0 && bladG == 0 && bladL == 0 && waropuzgorzam == 0 && waropuzgorotw == 0 && bladE == 0 && bladS == 0 && bladSS == 0 && bladSS_S == 0)
            {
                zastrafabram_gora = 1;
                brama2 = 1;
                zasczujniki_gorabr(1);
                czasopuznwlczujgora1 = czas;
                pwmwsilnkgor = 0;
                kieruneksilnikgor1(1);
                czasmaxotwugora1 = czas;
                czasminpradsilnikgorny = czas;
            }
        }
    }
    else
    {
        zasczujniki_gorabr(0);
        pwmwsilnkgor = 0;
        brama22 = 0;
        if (trybopuzgorzampilot == 0)
        {
            wyl_zas_gora();
            kieruneksilnikgor2(0); //////przekaznik lewo brama górna
        }
        else
        {
            waropuzgorzam = 1;
            czasopuzniwylprzekgorzam = czas;
        }
    }
}

void bramadolnazamykanie()
{

    if (brama1 == 0)
    {
        if (brama11 == 0)
        {
            if (bladB == 0 && bladG == 0 && bladL == 0 && waropuzdolzam == 0 && waropuzdolotw == 0 && bladF == 0 && bladD == 0 && bladR == 0 && bladRR == 0 && bladRR_R == 0)
            {
                zastrafabram_dol = 1;
                brama11 = 1;
                zasczujniki_dolnbr(1);
                czasopuznwlczujdol1 = czas;
                pwmwsilnkdol = 0;
                kieruneksilnikdol2(1);
                czasmaxzamudul1 = czas;
                czasminpradsilnikdolny = czas;
            }
        }
    }
    else
    {
        zasczujniki_dolnbr(0);
        pwmwsilnkdol = 0;
        brama1 = 0;
        if (trybopuzdolotwpilot == 0)
        {
            wyl_zas_dol();
            kieruneksilnikdol1(0); //////przekaznik prawo brama górna
        }
        else
        {
            waropuzdolotw = 1;
            czasopuzniwylprzekdolotw = czas;
        }
    }
}

void bramadolnaotwieranie()
{
    if (brama11 == 0)
    {
        if (brama1 == 0)
        {
            if (bladB == 0 && bladG == 0 && bladL == 0 && waropuzdolzam == 0 && waropuzdolotw == 0 && bladF == 0 && bladR == 0 && bladRR == 0 && bladRR_R == 0)
            {
                zastrafabram_dol = 1;
                brama1 = 1;
                zasczujniki_dolnbr(1);
                czasopuznwlczujdol1 = czas;
                pwmwsilnkdol = 0;
                kieruneksilnikdol1(1);
                czasmaxotwudul1 = czas;
                czasminpradsilnikdolny = czas;
            }
        }
    }
    else
    {
        zasczujniki_dolnbr(0);
        pwmwsilnkdol = 0;
        brama11 = 0;
        if (trybopuzdolzampilot == 0)
        {
            wyl_zas_dol();
            kieruneksilnikdol2(0); //////przekaznik lewo brama górna
        }
        else
        {
            waropuzdolzam = 1;
            czasopuzniwylprzekdolzam = czas;
        }
    }
}

void sterowanie_lampy(byte lampa, byte wlacz_wylacz)
{

    if (lampa == 0)
    { /// wyjscie zasilania lampy stojaca "3"  dom    #
        digitalWrite(lampa_stoj3_dom, wlacz_wylacz);
    }
    if (lampa == 1)
    { ////// PA2 // wyjscie zasilania lampy stojaca "2"  środek #
        digitalWrite(lampa_stoj2_srodek, wlacz_wylacz);
    }
    if (lampa == 2)
    { //// wyjscie zasilania lampy stojaca "1"  garaż
        digitalWrite(lampa_stoj1_garaz, wlacz_wylacz);
    }
    if (lampa == 3)
    { //// // wyjscie zasilania lampy wiszaca garaż   $
        digitalWrite(lampa_wisz_zew_garaz, wlacz_wylacz);
    }
    if (lampa == 4)
    { //// // wyjscie zasilania lampy wewnatrz garaż
        digitalWrite(lampa_wisz_wew_garaz, wlacz_wylacz);
    }
}

void animacja_wlaczenia_lamp(byte a, byte wl)
{ /// włączenie wieczorem (w odstępach czasu),wyłączenie rano.
    /// schemat animacji
    /*///////////////////////////////////////////////
    0 -> naraz wszytkie
    1 -> 012|3              właczanie od domu do garażu
    2 -> 021|3
    3 -> 210|3           włączanie noc/dzien/ po czsie tryb lamp 2 i 1
    4 -> 201|3
    5 -> 120|3
    6 -> 102|3

    */

    if (a == 0)
    {
        lampa_st_dom_0_1 = wl;
        lampa_st_sro_1_1 = wl;
        lampa_st_gar_2_1 = wl;
        if (wl_auto_wiszaca_noc == 1)
        {
            lampa_wi_gar_3_1 = wl;
        }
        war_wl_lamp_auto1 = 0;
        if (war_wl_lamp_auto2 == 2)
        {
            war_wl_lamp_auto3 = 0;
        }
        kroki_animacji = 0;
    }
    if (a == 1)
    {
        if (czas - czas_animacjilamp > czas_nastepnej_lampy_wl_wyl)
        {
            kroki_animacji++;
            czas_animacjilamp = czas;

            if (kroki_animacji == 1)
            {
                lampa_st_dom_0_1 = wl;
            }
            else
            {
                if (kroki_animacji == 2)
                {
                    lampa_st_sro_1_1 = wl;
                }
                else
                {
                    if (kroki_animacji == 3)
                    {
                        lampa_st_gar_2_1 = wl;
                    }
                    else
                    {
                        if (kroki_animacji == 4 && wl_auto_wiszaca_noc == 1)
                        {
                            lampa_wi_gar_3_1 = wl;
                        }
                        else
                        {
                            war_wl_lamp_auto1 = 0;
                            if (war_wl_lamp_auto2 == 2)
                            {
                                war_wl_lamp_auto3 = 0;
                            }
                            kroki_animacji = 0;
                        }
                    }
                }
            }
        }
    }
    if (a == 2)
    {
        if (czas - czas_animacjilamp > czas_nastepnej_lampy_wl_wyl)
        {
            kroki_animacji++;
            czas_animacjilamp = czas;

            if (kroki_animacji == 1)
            {
                lampa_st_dom_0_1 = wl;
            }
            else
            {
                if (kroki_animacji == 2)
                {
                    lampa_st_gar_2_1 = wl;
                }
                else
                {
                    if (kroki_animacji == 3)
                    {
                        lampa_st_sro_1_1 = wl;
                    }
                    else
                    {
                        if (kroki_animacji == 4 && wl_auto_wiszaca_noc == 1)
                        {
                            lampa_wi_gar_3_1 = wl;
                        }
                        else
                        {
                            war_wl_lamp_auto1 = 0;
                            if (war_wl_lamp_auto2 == 2)
                            {
                                war_wl_lamp_auto3 = 0;
                            }
                            kroki_animacji = 0;
                        }
                    }
                }
            }
        }
    }
    if (a == 3)
    {
        if (czas - czas_animacjilamp > czas_nastepnej_lampy_wl_wyl)
        {
            kroki_animacji++;
            czas_animacjilamp = czas;

            if (kroki_animacji == 1)
            {
                lampa_st_gar_2_1 = wl;
            }
            else
            {
                if (kroki_animacji == 2)
                {
                    lampa_st_sro_1_1 = wl;
                }
                else
                {
                    if (kroki_animacji == 3)
                    {
                        lampa_st_dom_0_1 = wl;
                    }
                    else
                    {
                        if (kroki_animacji == 4 && wl_auto_wiszaca_noc == 1)
                        {
                            lampa_wi_gar_3_1 = wl;
                        }
                        else
                        {
                            war_wl_lamp_auto1 = 0;
                            if (war_wl_lamp_auto2 == 2)
                            {
                                war_wl_lamp_auto3 = 0;
                            }
                            kroki_animacji = 0;
                        }
                    }
                }
            }
        }
    }
    if (a == 4)
    {
        if (czas - czas_animacjilamp > czas_nastepnej_lampy_wl_wyl)
        {
            kroki_animacji++;
            czas_animacjilamp = czas;
            if (kroki_animacji == 1)
            {
                lampa_st_gar_2_1 = wl;
            }
            else
            {
                if (kroki_animacji == 2)
                {
                    lampa_st_dom_0_1 = wl;
                }
                else
                {
                    if (kroki_animacji == 3)
                    {
                        lampa_st_sro_1_1 = wl;
                    }
                    else
                    {
                        if (kroki_animacji == 4 && wl_auto_wiszaca_noc == 1)
                        {
                            lampa_wi_gar_3_1 = wl;
                        }
                        else
                        {
                            war_wl_lamp_auto1 = 0;
                            if (war_wl_lamp_auto2 == 2)
                            {
                                war_wl_lamp_auto3 = 0;
                            }
                            kroki_animacji = 0;
                        }
                    }
                }
            }
        }
    }
    if (a == 5)
    {
        if (czas - czas_animacjilamp > czas_nastepnej_lampy_wl_wyl)
        {
            kroki_animacji++;
            czas_animacjilamp = czas;

            if (kroki_animacji == 1)
            {
                lampa_st_sro_1_1 = wl;
            }
            else
            {
                if (kroki_animacji == 2)
                {
                    lampa_st_gar_2_1 = wl;
                }
                else
                {
                    if (kroki_animacji == 3)
                    {
                        lampa_st_dom_0_1 = wl;
                    }
                    else
                    {
                        if (kroki_animacji == 4 && wl_auto_wiszaca_noc == 1)
                        {
                            lampa_wi_gar_3_1 = wl;
                        }
                        else
                        {
                            war_wl_lamp_auto1 = 0;
                            if (war_wl_lamp_auto2 == 2)
                            {
                                war_wl_lamp_auto3 = 0;
                            }
                            kroki_animacji = 0;
                        }
                    }
                }
            }
        }
    }
    if (a == 6)
    {
        if (czas - czas_animacjilamp > czas_nastepnej_lampy_wl_wyl)
        {
            kroki_animacji++;
            czas_animacjilamp = czas;

            if (kroki_animacji == 1)
            {
                lampa_st_sro_1_1 = wl;
            }
            else
            {
                if (kroki_animacji == 2)
                {
                    lampa_st_dom_0_1 = wl;
                }
                else
                {
                    if (kroki_animacji == 3)
                    {
                        lampa_st_gar_2_1 = wl;
                    }
                    else
                    {
                        if (kroki_animacji == 4 && wl_auto_wiszaca_noc == 1)
                        {
                            lampa_wi_gar_3_1 = wl;
                        }
                        else
                        {
                            war_wl_lamp_auto1 = 0;
                            if (war_wl_lamp_auto2 == 2)
                            {
                                war_wl_lamp_auto3 = 0;
                            }

                            kroki_animacji = 0;
                        }
                    }
                }
            }
        }
    }
}

void animacja_wlaczenia_lamp_czujniki1(byte a, byte wl)
{
    /*/////////////////////////////////////////////////

       1 -> 3210             właczanie od garażu do domu
       2 -> 0123             właczanie od domu do garażu

    ////animacje z czujników */
    if (a == 1) // GARAZ -->> DOM
    {
        if (czas - czas_animacjilamp_1 > czas_nastepnej_lampy_wl_wyl_czujniki)
        {
            kroki_animacji_1++;
            czas_animacjilamp_1 = czas;
            //

            if (kroki_animacji_1 == 1)
            {
                lampa_wi_gar_3_2 = wl;
            }
            else
            {
                if (kroki_animacji_1 == 2)
                {
                    lampa_st_gar_2_2 = wl;
                }
                else
                {
                    if (kroki_animacji_1 == 3)
                    {
                        lampa_st_sro_1_2 = wl;
                    }
                    else
                    {
                        if (kroki_animacji_1 == 4)
                        {
                            lampa_st_dom_0_2 = wl;
                        }
                        else
                        {
                            if (war_przyjazd_droga == 2)
                            {
                                war_przyjazd_droga = 0;
                            }

                            if (war_przyjazd_droga == 1)
                            {
                                war_przyjazd_droga = 2;
                                czas_wyl_lam_wisz_licz = czas; /// czasy podtrzymania wyłączenia przy właczaniu
                                czas_wyl_lam_sto_licz = czas;
                            }

                            if (war_lampa_tylko_dom == 3)
                            {
                                war_lampa_tylko_dom = 0;
                            }

                            if (war_lampa_tylko_dom == 2)
                            {
                                czas_wyl_lam_sto_licz = czas;
                                czas_wyl_lam_wisz_licz = czas;
                                war_lampa_tylko_dom = 3;
                            }

                            kroki_animacji_1 = 0;
                        }
                    }
                }
            }
        }
    }
    if (a == 2) ///   DOM -->> GARAZ
    {
        if (czas - czas_animacjilamp_1 > czas_nastepnej_lampy_wl_wyl_czujniki)
        {
            kroki_animacji_1++;
            czas_animacjilamp_1 = czas;

            if (kroki_animacji_1 == 1)
            {
                lampa_st_dom_0_2 = wl;
            }
            else
            {
                if (kroki_animacji_1 == 2)
                {
                    lampa_st_sro_1_2 = wl;
                }
                else
                {
                    if (kroki_animacji_1 == 3)
                    {
                        lampa_st_gar_2_2 = wl;
                    }
                    else
                    {
                        if (kroki_animacji_1 == 4)
                        {
                            lampa_wi_gar_3_2 = wl;
                        }
                        else
                        {
                            if (war_przyjazd_droga == 2)
                            {
                                war_przyjazd_droga = 0;
                            }

                            if (war_przyjazd_droga == 1)
                            {
                                war_przyjazd_droga = 2;
                                czas_wyl_lam_wisz_licz = czas; /// czasy podtrzymania wyłączenia przy właczaniu
                                czas_wyl_lam_sto_licz = czas;
                            }

                            if (war_lampa_tylko_dom == 3)
                            {
                                war_lampa_tylko_dom = 0;
                            }

                            if (war_lampa_tylko_dom == 2)
                            {
                                czas_wyl_lam_sto_licz = czas;
                                czas_wyl_lam_wisz_licz = czas;
                                war_lampa_tylko_dom = 3;
                            }

                            kroki_animacji_1 = 0;
                        }
                    }
                }
            }
        }
    }
}

/// po wywołaniu tego reaguja bramy //główna funkcja do sterowania
void komenda_otworz_dolna_brama()
{
    if (aktybrama1 == 1)
    {
        czaspikaczu1 = czas;
        if (otworzdolnazopuznienie == 0)
        {
            bramadolnaotwieranie();
        }
        else
        {
            otworzdolnazopuznienie = 0;
            if (otworzgorazopuznienie == 0)
            {
                pcf8574(cyfrybledow[0]);
            }
        }
    }
}

void komenda_zamknij_dolna_brama()
{
    if (aktybrama1 == 1)
    {
        czaspikaczu1 = czas;
        if (otworzdolnazopuznienie == 0)
        {
            bramadolnazamykanie();
        }
        else
        {
            otworzdolnazopuznienie = 0;
            if (otworzgorazopuznienie == 0)
            {
                pcf8574(cyfrybledow[0]);
            }
        }
    }
}

void komenda_otworz_gorna_brama()
{
    if (aktybrama2 == 1)
    {
        czaspikaczu1 = czas;
        if (otworzgorazopuznienie == 0)
        {
            bramagornaotwieranie();
        }
        else
        {
            otworzgorazopuznienie = 0;
            if (otworzdolnazopuznienie == 0)
            {
                pcf8574(cyfrybledow[0]);
            }
        }
    }
}

void komenda_zamknij_gorna_brama()
{
    if (aktybrama2 == 1)
    {
        czaspikaczu1 = czas;
        if (otworzgorazopuznienie == 0)
        {
            bramagornazamykanie();
        }
        else
        {
            otworzgorazopuznienie = 0;
            if (otworzdolnazopuznienie == 0)
            {
                pcf8574(cyfrybledow[0]);
            }
        }
    }
}
//////////////////////////////////////////////////////////////////
void off_forewer_gorna_czujniki_szyny()
{
    var_stanu_szyna_gorna = 0;
    czas_wys_bl_sw_gorny = czas;
    bladS = 0;
    bladSS = 0;
    bladSS_S = 37;
    blad = 1;
    brama2 = 0;
    brama22 = 0;
    pwmwsilnkgor = 0;
    zasczujniki_gorabr(0);
    kieruneksilnikgor2(0); //////przekaznik lewo brama d
    kieruneksilnikgor1(0); //////przekaznik prawo brama d
    wyl_zas_gora();
}

void off_forewer_dolna_czujniki_szyny()
{
    var_stanu_szyna_dolna = 0;
    czas_wys_bl_sw_dolny = czas;
    bladR = 0;
    bladRR = 0;
    bladRR_R = 9;
    blad = 1;
    brama1 = 0;
    brama11 = 0;
    pwmwsilnkdol = 0;
    zasczujniki_dolnbr(0);
    kieruneksilnikdol2(0); //////przekaznik lewo brama d
    kieruneksilnikdol1(0); //////przekaznik prawo brama d
    wyl_zas_dol();
}

void writeLongIntoEEPROM(int address, unsigned long number)
{
    EEPROM.update(address, (number >> 24) & 0xFF);
    EEPROM.update(address + 1, (number >> 16) & 0xFF);
    EEPROM.update(address + 2, (number >> 8) & 0xFF);
    EEPROM.update(address + 3, number & 0xFF);
}

unsigned long readLongFromEEPROM(int address)
{
    return ((unsigned long)EEPROM.read(address) << 24) + ((unsigned long)EEPROM.read(address + 1) << 16) + ((unsigned long)EEPROM.read(address + 2) << 8) + (unsigned long)EEPROM.read(address + 3);
}

void odczyt_Pamieci_EEprom()
{

    for (int wiersze_pilot_433 = 0; wiersze_pilot_433 <= 5; wiersze_pilot_433++)
    {
        if (wiersze_pilot_433 == 0)
        {
            nr_dwa_eeprom_pilot_433 = 0;
        }
        if (wiersze_pilot_433 == 1)
        {
            nr_dwa_eeprom_pilot_433 = 16;
        }
        if (wiersze_pilot_433 == 2)
        {
            nr_dwa_eeprom_pilot_433 = 32;
        }
        if (wiersze_pilot_433 == 3)
        {
            nr_dwa_eeprom_pilot_433 = 64;
        }
        if (wiersze_pilot_433 == 4)
        {
            nr_dwa_eeprom_pilot_433 = 128;
        }
        if (wiersze_pilot_433 == 5)
        {
            nr_dwa_eeprom_pilot_433 = 256;
        }

        for (int kolumny_pilot_433 = 0; kolumny_pilot_433 <= 3; kolumny_pilot_433++)
        {

            if (kolumny_pilot_433 == 0)
            {
                nr_eeprom_pilot_433 = 0;
            }
            if (kolumny_pilot_433 == 1)
            {
                nr_eeprom_pilot_433 = 4;
            }
            if (kolumny_pilot_433 == 2)
            {
                nr_eeprom_pilot_433 = 8;
            }
            if (kolumny_pilot_433 == 3)
            {
                nr_eeprom_pilot_433 = 12;
            }

            tabRCswitch[wiersze_pilot_433][kolumny_pilot_433] = readLongFromEEPROM((nr_eeprom_pilot_433 + nr_dwa_eeprom_pilot_433));
        }
    }
}

void setup()
{
    pinMode(buzzer, OUTPUT);

    /// brama dolna //
    pinMode(silnikdolnalewo, OUTPUT);
    pinMode(silnikdolnaprawo, OUTPUT);
    pinMode(pwmsilnikdol, OUTPUT);
    pinMode(wejsciefoto1dol, INPUT_PULLUP);
    pinMode(amperilnikdolnab, INPUT);
    pinMode(przycbragardolna, INPUT);
    pinMode(wejsciehaLL11, INPUT_PULLUP);
    pinMode(wejsciehaLL12, INPUT_PULLUP);
    pinMode(switch_dolna_szyna, INPUT_PULLUP);
    // wspulne///
    pinMode(zastrafasilnkibr, OUTPUT);
    pinMode(czujnikzasilnik, INPUT_PULLUP);
    /// brama gorna //
    pinMode(silnikgornalewo, OUTPUT);
    pinMode(silnikgornaprawo, OUTPUT);
    pinMode(pwmsilnikgora, OUTPUT);
    pinMode(wejsciefoto1gor, INPUT_PULLUP);
    pinMode(amperilnikgornab, INPUT);
    pinMode(przycbragargorna, INPUT);
    pinMode(wejsciehaLL21, INPUT_PULLUP);
    pinMode(wejsciehaLL22, INPUT_PULLUP);
    pinMode(switch_gorna_szyna, INPUT_PULLUP);
    ///

    pinMode(zasilanieczujkaru, OUTPUT);

    // lampy
    pinMode(przycoswietlenie, INPUT);
    pinMode(lampa_stoj1_garaz, OUTPUT);
    pinMode(lampa_stoj2_srodek, OUTPUT);
    pinMode(lampa_stoj3_dom, OUTPUT);
    pinMode(lampa_wisz_zew_garaz, OUTPUT);
    pinMode(lampa_wisz_wew_garaz, OUTPUT);

    // czujniki
    pinMode(wejczujnikruchu, INPUT_PULLUP);
    pinMode(czujnikfotrezys, INPUT);
    pinMode(czujrodzajzasil, INPUT_PULLUP);

    // buzer
    pinMode(buzzer, OUTPUT);

    if (monitoring == 1)
    {
        Serial.begin(115200); // uart na pinach programowania ;
    }

    // rs486
    // Serial1.begin(115200); //uart na pinach Rs485
    pinMode(WYBOR_RS485, OUTPUT);
    digitalWrite(WYBOR_RS485, LOW); // odbiornik

    // SPI.begin();

    Wire.begin();
    pcf8574(cyfrybledow[0]);

    radio.begin();
    radio.setAutoAck(false);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.openReadingPipe(1, pipe);
    radio.startListening();
    radio.powerUp();

    IrReceiver.begin(podczerwien_ir);
    // stare TO JEST JAKIES BŁEDNE
    //      TCCR3A |= (1 << WGM31) | (1 << WGM30);
    //     TCCR3A |= (1 << COM3B1) | (1 << COM3C1) | (1 << COM3B0) | (1 << COM3C0);
    //    TCCR3B |= (1 << CS30);

    // TCCR3A |= (1 << WGM31) | (1 << WGM30) | (1 << COM3B1) | (1 << COM3C1) | (1 << COM3B0) | (1 << COM3C0);
    //       TCCR3B |= (1 << CS30);

    // nowe
    TCCR3A = 0b00111111; // 10 bit bez przeskalera poiprawny fazowo
    TCCR3B = 0b00000001;

    mySwitch.enableReceive(inter_433hzm); // Receiver on interrupt 0 => that is pin #2

    /// odczyt z pamięci eeprom //////
    /// nr_pilota  /// 1,2,3,4,5,6// cyfrowo: 0,16,32,64,128,256
    odczyt_Pamieci_EEprom();

    for (int wl = 10; wl >= 0; wl--)
    {
        pcf8574(cyfrybledow[wl]);
        delay(300);
    }
}

void loop()
{
    czas = millis(); // czas

    ////czujnikrodzaju zasilania /////////////////////////////////////////////////////////////////////
    rodzaj_zasilania = digitalRead(czujrodzajzasil); /////0--> siec // 1-->akumulator

    // zmien_czuj_zas = digitalRead(czujnikzasilnik);   /////0--> jest napiecie // 1-->nie ma napiecia  na silnikach
    if (digitalRead(czujnikzasilnik) == 1)
    {
        if (czas - czas_odlicz_czujka_silnikizasilania > 2400)
        {
            zmien_czuj_zas = 1;
        }
    }
    else
    {
        zmien_czuj_zas = 0;
        czas_odlicz_czujka_silnikizasilania = czas;
    }

    if (czas < 7)
    {
        czas_odlicz_czujka_silnikizasilania = czas;
        czas_oudznia_auozamknij_licz = czas;
        czas_wyl_lam_sto_licz = czas;
        czas_wylacz_reczne_lampy_licz = czas;
        czas_przyc_licz = czas;
        czas_lampa_tylko_dom_licz = czas;
        czas_opuz_czujniki_licz = czas;
        czas_animacjilamp = czas;
        czas_animacjilamp_1 = czas;
        czas_przelaczenia = czas;
        czaspikaczu1 = czas;
        czaspikaczu2 = czas;
        czaspikaczu3 = czas;
        czasanimacji = czas;
        czasanimacji_pilot433 = czas;
        czasbledu = czas;
        czaspomiarfotorezystor = czas;
        czasprzyczamykaniedolnaanimacja = czas;
        czasprzyczamykaniedolna = czas;
        czaspomiarampdolna = czas;
        czaspwmplussilnikdolna = czas;
        czasopuznwlczujdol1 = czas;
        czaswysblfotodolny = czas;
        czaswysblhalldolny = czas;
        czasmaxotwudul1 = czas;
        czasmaxzamudul1 = czas;
        czaswysbladNtwdul = czas;
        czaswysbladzamdol = czas;
        czasopuzniwylprzekdolotw = czas;
        czasopuzniwylprzekdolzam = czas;
        czaswysbladpraddul = czas;
        czasminpradsilnikdolny = czas;
        czasbladpraddolny = czas;
        czaswlaczeniazass1 = czas;
        czaswysbladwlzasil = czas;
        czaswysbladwylzasil = czas;
        czaswylaczeniazass1 = czas;
        czasprzyczamykaniegoraanimacja = czas;
        czasprzyczamykaniegora = czas;
        czaspomiarampgorna = czas;
        czaspwmplussilnikgora = czas;
        czasopuznwlczujgora1 = czas;
        czaswysblfotogorny = czas;
        czaswysblhallgorny = czas;
        czasmaxotwugora1 = czas;
        czasmaxzamugora1 = czas;
        czaswysbladNtwgora = czas;
        czaswysbladzamgora = czas;
        czasopuzniwylprzekgorzam = czas;
        czasopuzniwylprzekgorotw = czas;
        czasbladpradgorny = czas;
        czaswysbladpradgora = czas;
        czasminpradsilnikgorny = czas;
        czas_wyl_lam_wisz_licz = czas;
        czas_zabezpiecz_czujka_dom_licz = czas;

        czas_wys_bl_sw_dolny = czas;
        czas_wys_bl_sw_gorny = czas;
        czas_wylna_srodku_dol = czas;
        czas_wylna_srodku_gor = czas;

        czas_zwloki_brdolna_otworz_prad_licz = czas;
        czas_zwloki_brdolna_zamknij_prad_licz = czas;
        czas_zwloki_brgorna_otworz_prad_licz = czas;
        czas_zwloki_brgorna_zamknij_prad_licz = czas;

        czas_filtr_przyc_dolna1 = czas;
        czas_filtr_przyc_dolna2 = czas;
        czas_filtr_przyc_gorna1 = czas;
        czas_filtr_przyc_gorna2 = czas;

        czasprzycotwieraniedolna = czas;
        czasprzycotwieraniegora = czas;
        czas_pilot_czas_ustawien_licz = czas;
        ////////////////////////do uzupełnienia !!!!!!!!/////////////
    }
    ///////czujnik zmierzchu/////////////////////////////////////////////////////////////////////////////////////

    if (czas - czaspomiarfotorezystor > (unsigned long)3000) // co 3 sek dla 100ms // co 15 min dla 30ms i 3000 pomiarom//
    {
        if (liczbapomiarfotorez < 200)                                       //-->liczba pomiarów
        {                                                                    // wyswietlenie pomiaru co 10min
            odczytfotorezys = odczytfotorezys + analogRead(czujnikfotrezys); // wejscie czujnik fotorezystora
            liczbapomiarfotorez++;
        }
        else
        {
            adczmierzch = odczytfotorezys / 200; /// zwraca adczmierzch jako wartosc 12bit jasnosci
            odczytfotorezys = 0;
            liczbapomiarfotorez = 0;
        }
        czaspomiarfotorezystor = czas;
    }

    // adczmierzch = 900; /////test///////////////////////!!!!!!!!!!!!!!!!!

    //////////////////Lampay//////////////////////////////////////////////////////// lampy//////////////////////////////////////

    ////NOC RAZ TYLKO//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (adczmierzch > wartoscdonocy && noc == 0)
    {
        noc = 1;
        //////////////////////////////Auto zamykanie po wykryciu nocy //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (zamykaniewnocy == 1)
        {
            czas_oudznia_auozamknij_licz = czas;
            war_czas_autozamk = 1;
        }

        //////////////////////////////koniec auto zamykania po wykryciu nocy //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //
        if (rodzaj_zasilania == 0)
        {
            if (tryb_lamp_auto == 1) /// włączenie wieczorem (w odstępach czasu),wyłączenie rano.
            {
                war_wl_lamp_auto1 = 1;
                nr_animacji = random(7);
            }

            if (tryb_lamp_auto == 2)
            {
                war_wl_lamp_auto1 = 1;
                war_wl_lamp_auto2 = 1;
                nr_animacji = random(7);
                czas_wl_lamp_wiecor_licz = czas;
            }
        }
        //
    }
    ///////////////////////koniec noc raz////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (czas - czas_oudznia_auozamknij_licz > czas_oudznia_auozamknij && war_czas_autozamk == 1)
    {
        war_czas_autozamk = 0;
        czaspikaczu2 = czas;
        if (aktybrama1 == 1)
        {
            bramadolnazamykanie(); //////////zamknij brama dol 1 /////////
        }

        if (aktybrama2 == 1)
        {
            bramagornazamykanie(); //////////zamknij 2 brama góra
        }
    }

    if (noc == 1)
    {
        if (war_wl_lamp_auto1 == 1) ////włączenie lamp na czas i na noc
        {
            animacja_wlaczenia_lamp(nr_animacji, 1);
        }
    }
    else
    {
        if (war_wl_lamp_auto1 == 1) ////wyłączenie lamp na czas i na noc
        {
            animacja_wlaczenia_lamp(nr_animacji, 0);
        }
    }

    ////////////////////////wyłączenie lamp po czasie/////////////////////////////////////////////

    if (czas - czas_wl_lamp_wiecor_licz > czas_wl_lamp_wiecor)
    {
        if (war_wl_lamp_auto2 == 1)
        {
            nr_animacji = random(7);
            war_wl_lamp_auto2 = 2;
            war_wl_lamp_auto3 = 1;
        }
    }

    if (war_wl_lamp_auto2 == 2 && war_wl_lamp_auto3 == 1)
    {
        animacja_wlaczenia_lamp(nr_animacji, 0);
    }
    //////////////////////////////////////////////////////////////////////////////////////////

    //////////////jest ddzien // RAZ TYLKO//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (adczmierzch < wartoscdonocy && noc == 1)
    {
        lampa_st_dom_0_3 = 0;
        lampa_wiw_gar_4_2 = 0;
        lampa_wiw_gar_4_1 = 0;
        lampa_st_dom_0_2 = 0;
        lampa_st_sro_1_2 = 0;
        lampa_st_gar_2_2 = 0;
        lampa_wi_gar_3_2 = 0;
        noc = 0;

        if (tryb_lamp_auto == 1) /// wyłączenie wieczorem (w odstępach czasu),wyłączenie rano.
        {
            nr_animacji = random(7);
            war_wl_lamp_auto1 = 1;
        }
    }
    ////////////////////CZUJNIKI ruchu i lampy ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (czas - czas_opuz_czujniki_licz > czas_opuz_czujniki && noc == 1 && rodzaj_zasilania == 0)
    {
        czujnik_garaz = digitalRead(wejczujnikruchu);
        //////////////////////////////////////////////////// czujnik garazu////////////////////////////////////////////////
        if (czujnik_garaz == 0)
        {
            ////przyjazd do domu od strony garażu () pentla

            if (war_lampa_tylko_dom == 0)
            {
                // włącza lampe wiszacą garażową jesli jest wyłaczana w połowie a nagle czujnik garazu łapie
                if (czas - czas_wyl_lam_wisz_licz > czas_wyl_lam_wisz && wyl_lamp_wisz_przyjazd == 1 && czas - czas_wyl_lam_sto_licz <= czas_wyl_lam_sto && war_przyjazd_droga == 2)
                {
                    lampa_wi_gar_3_2 = 1;
                }

                //// włączenie animajcji w strone domu
                if (war_przyjazd_droga == 0)
                {
                    wybor_animacji = 1;
                    kroki_animacji_1 = 0;
                    war_przyjazd_droga = 1;
                }

                if (czas - czas_wyl_lam_sto_licz > czas_wyl_lam_sto && war_przyjazd_droga == 2) // przy wyłączaniu w strone domu , po wykryciu czujki garażu wraca (jesli nie zgasły w strone domu  ) do  garau
                {
                    wybor_animacji = 2;
                    kroki_animacji_1 = 0;
                    war_przyjazd_droga = 1;
                    if (lampa_wi_gar_3_2 == 1)
                    {
                        kroki_animacji_1++;
                    }
                    if (lampa_st_gar_2_2 == 1)
                    {
                        kroki_animacji_1++;
                    }
                    if (lampa_st_sro_1_2 == 1)
                    {
                        kroki_animacji_1++;
                    }
                    if (lampa_st_dom_0_2 == 1)
                    {
                        kroki_animacji_1++;
                    }
                }
            }
            else
            {

                if (czas - czas_wyl_lam_wisz_licz > czas_wyl_lam_wisz && wyl_lamp_wisz_przyjazd == 1 && czas - czas_wyl_lam_sto_licz <= czas_wyl_lam_sto && war_lampa_tylko_dom == 3)
                { // włącza lampe wiszacą garażową jesli jest wyłaczana w połowie a nagle czujnik garazu łapie
                    lampa_wi_gar_3_2 = 1;
                }

                if (czas - czas_wyl_lam_sto_licz > czas_wyl_lam_sto && war_lampa_tylko_dom == 3) //// (włączenie animajcji w strone garazu ) ALE WŁĄCZENIE ANIMACJI OD GARAZU DO DOMU W CELU PODTRZYMANIA PODCZAS WYŁAĆZANIA
                {
                    wybor_animacji = 1;
                    kroki_animacji_1 = 0;
                    war_lampa_tylko_dom = 2;
                    if (lampa_wi_gar_3_2 == 1)
                    {
                        kroki_animacji_1++;
                    }
                    if (lampa_st_gar_2_2 == 1)
                    {
                        kroki_animacji_1++;
                    }
                    if (lampa_st_sro_1_2 == 1)
                    {
                        kroki_animacji_1++;
                    }
                    if (lampa_st_dom_0_2 == 1)
                    {
                        kroki_animacji_1++;
                    }
                }
            }

            if (war_lampa_tylko_dom == 1)
            {
                wybor_animacji = 2;
                kroki_animacji_1 = 1;
                war_lampa_tylko_dom = 2;
            }

            czas_wyl_lam_wisz_licz = czas;
            czas_wyl_lam_sto_licz = czas;
        }

        if (war_przyjazd_droga == 1 || war_lampa_tylko_dom == 2)
        {
            animacja_wlaczenia_lamp_czujniki1(wybor_animacji, 1);
        }

        if (war_przyjazd_droga == 2)
        {

            if (czas - czas_wyl_lam_wisz_licz > czas_wyl_lam_wisz && wyl_lamp_wisz_przyjazd == 1)
            {
                lampa_wi_gar_3_2 = 0;
            }

            if (czas - czas_wyl_lam_sto_licz > czas_wyl_lam_sto)
            {
                animacja_wlaczenia_lamp_czujniki1(1, 0);
            }
        }

        if (war_lampa_tylko_dom == 3)
        {
            if (czas - czas_wyl_lam_wisz_licz > czas_wyl_lam_wisz && wyl_lamp_wisz_przyjazd == 1)
            {
                lampa_wi_gar_3_2 = 0;
            }

            if (czas - czas_wyl_lam_sto_licz > czas_wyl_lam_sto)
            {
                animacja_wlaczenia_lamp_czujniki1(2, 0);
            }
        }

        ////////////////////////////////////////////////CZUJNIK DOMU//////////////////////////////////////////////////////////
        if (czujnik_dom == 1) //// czujnik domu  () pentla
        {

            if (war_przyjazd_droga == 2 && war_wyl_lwisz_przyjsciedom == 1 && czujnik_garaz == 1)
            { // jesli przyjdziemy do domu a czujnik garazu nie łapie to wyłączy lampe wiszaca na garazu jesli war_wyl_lwisz_przyjsciedom==1 jesli 0 -> nie wyłaczy
                lampa_wi_gar_3_2 = 0;
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            //
            /////////logika sterowania lamp  wyjscie z domu  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if (war_lampa_tylko_dom == 0 && war_przyjazd_droga == 0)
            {
                war_lampa_tylko_dom = 1;
                lampa_st_dom_0_2 = 1;
            }

            lampa_st_dom_0_3 = 1;
            czas_lampa_tylko_dom_licz = czas;

            if (czas - czas_zabezpiecz_czujka_dom_licz > czas_zabezpiecz_czujka_dom)
            { // zabezpieczenie jakby czujka ruchu domu nie wyłaczyła
                czujnik_dom = 0;
            }
        }
        else
        {
            czas_zabezpiecz_czujka_dom_licz = czas; // zabezpieczenie jakby czujka ruchu domu nie wyłaczyła
        }

        /* if (czas - czas_lampa_tylko_dom_licz > czas_lampa_tylko_dom && war_lampa_tylko_dom == 1)
         { // jesli czujnik garazu nie złapie to lampy dom wyłaczą sie po czasie czas_lampa_tylko_dom
            lampa_st_dom_0_2 = 0;
            //   lampa_st_dom_0_3 = 0;
             war_lampa_tylko_dom = 0;
         }
         */

        if (czas - czas_lampa_tylko_dom_licz > czas_lampa_tylko_dom)
        { // jesli czujnik garazu nie złapie to lampy dom wyłaczą sie po czasie czas_lampa_tylko_dom
            lampa_st_dom_0_3 = 0;

            if (war_lampa_tylko_dom == 1)
            {
                lampa_st_dom_0_2 = 0;
                war_lampa_tylko_dom = 0;
            }
        }
    }

    ////////////////////////////////////////////////////////koniec lamp//////////////////////////////////////

    //////////////////przyciski lamp /////////////////////////////////////////////////////////////////////
    if (przyciski_oswietlenie == 1 && rodzaj_zasilania == 0)
    {
        adcprzycoswietlenie = analogRead(przycoswietlenie);
        // tolerancaj +-15j
        // zakresy|||                                <0|100>  <221|251>  <349|379>  <490|520>
        // przyciski                                    0        1          2           3

        if (adcprzycoswietlenie < 100)
        {
            warprzyc_wewn = 0;
            warprzyc_stoj = 0;
        }
        else
        {
            if (czas - czas_przyc_licz > czas_przyc) /// np 16ms
            {
                if (490 < adcprzycoswietlenie && adcprzycoswietlenie < 520 && warprzyc_stoj == 0) ////  505 / 2,466V////logika przyciki lampy stojacych
                {
                    if (tryb_ustawien_433 <= 1)
                    {
                        czas_przyc_licz = czas;
                        czas_wylacz_reczne_lampy_licz = czas;
                        stan_lampa_stojaca++;
                        if (stan_lampa_stojaca == 4)
                        {
                            stan_lampa_stojaca = 1;
                        }
                        if (stan_lampa_stojaca == 1 || stan_lampa_stojaca == 2)
                        {
                            czaspikaczu1 = czas;
                        }
                        if (stan_lampa_stojaca == 3)
                        {
                            czaspikaczu2 = czas;
                        }
                    }
                    else
                    {
                        czaspikaczu1 = czas;
                        // 0,16,32,64,128,256

                        nr_pilota_licz_plus++;

                        if (nr_pilota_licz_plus > 5)
                        {
                            nr_pilota_licz_plus = 0;
                        }

                        if (nr_pilota_licz_plus == 0)
                        {
                            nr_pilota = 0;
                        }
                        else if (nr_pilota_licz_plus == 1)
                        {
                            nr_pilota = 16;
                        }
                        else if (nr_pilota_licz_plus == 2)
                        {
                            nr_pilota = 32;
                        }
                        else if (nr_pilota_licz_plus == 3)
                        {
                            nr_pilota = 64;
                        }
                        else if (nr_pilota_licz_plus == 4)
                        {
                            nr_pilota = 128;
                        }
                        else if (nr_pilota_licz_plus == 5)
                        {
                            nr_pilota = 256;
                        }
                    }

                    warprzyc_stoj = 1;
                }
                else
                {
                    if (221 < adcprzycoswietlenie && adcprzycoswietlenie < 251 && warprzyc_wewn == 0) // 236 / 1,154v      ////logika przyciki lampy wewnętrznej
                    {
                        if (tryb_ustawien_433 <= 1)
                        {
                            czas_przyc_licz = czas;
                            czas_wylacz_reczne_lampy_licz = czas;
                            stan_lampa_wewntrzna++;
                            if (stan_lampa_wewntrzna == 4)
                            {
                                stan_lampa_wewntrzna = 1;
                            }
                            if (stan_lampa_wewntrzna == 1 || stan_lampa_wewntrzna == 2)
                            {
                                czaspikaczu1 = czas;
                            }
                            if (stan_lampa_wewntrzna == 3)
                            {
                                czaspikaczu2 = czas;
                            }
                        }
                        else
                        { // wyjście z trybu programowania pilotów 433

                            if (tryb_ustawien_433 == 2)
                            {
                                nr_pilota = 0;
                                czaspikaczu2 = czas;
                                tryb_ustawien_433 = 0;
                                odczyt_Pamieci_EEprom();
                                pcf8574(cyfrybledow[0]);
                            }

                            if (tryb_ustawien_433 > 2)
                            {
                                tryb_ustawien_433 = 2;
                                czaspikaczu1 = czas;
                                czas_pilot_czas_ustawien_licz = czas;
                            }
                        }
                        warprzyc_wewn = 1;
                    }
                }
            }
        }

        /////////auto powrót do sterowania automatyczego////
        if (czas - czas_wylacz_reczne_lampy_licz > czas_wylacz_reczne_lampy)
        {
            if (stan_lampa_wewntrzna != 3)
            {
                czaspikaczu2 = czas;
                stan_lampa_wewntrzna = 3;
            }
            if (stan_lampa_stojaca != 3)
            {
                czaspikaczu2 = czas;
                stan_lampa_stojaca = 3;
            }
        }
    }
    //
    /////////////////////////////// LOGIKA WYJSCIA NA LAMPY///////////////////////////////////////////////////
    //
    /////////////////////////////// wyjscie pinów///////////////////////////////////////////////////
    if (rodzaj_zasilania == 0)
    {
        if (stan_lampa_stojaca == 3)
        {
            ///////////////////lampa wiszaca zewnetrzna/////////////////////////////////////////////////////////////////////////////////////////////////////
            if (lampa_wi_gar_3_1 == 1 || lampa_wi_gar_3_2 == 1)
            {
                sterowanie_lampy(3, 1);
            }

            if (lampa_wi_gar_3_1 == 0 && lampa_wi_gar_3_2 == 0)
            {
                sterowanie_lampy(3, 0);
            }
            ///////////////////stojace ///////////////////////////////////////////////////////////////////////////////////////////////////////
            if (lampa_st_dom_0_1 == 1 || lampa_st_dom_0_2 == 1 || lampa_st_dom_0_3 == 1)
            {
                sterowanie_lampy(0, 1);
            }
            if (lampa_st_dom_0_1 == 0 && lampa_st_dom_0_2 == 0 && lampa_st_dom_0_3 == 0)
            {
                sterowanie_lampy(0, 0);
            }

            /////////////////////////////////////////////////////////////////////////////////////////
            if (lampa_st_sro_1_1 == 1 || lampa_st_sro_1_2 == 1)
            {
                sterowanie_lampy(1, 1);
            }
            if (lampa_st_sro_1_1 == 0 && lampa_st_sro_1_2 == 0)
            {
                sterowanie_lampy(1, 0);
            }

            /////////////////////////////////////////////////////////////////////////////////////////
            if (lampa_st_gar_2_1 == 1 || lampa_st_gar_2_2 == 1)
            {
                sterowanie_lampy(2, 1);
            }
            if (lampa_st_gar_2_1 == 0 && lampa_st_gar_2_2 == 0)
            {
                sterowanie_lampy(2, 0);
            }
        }
        else
        {
            if (stan_lampa_stojaca == 2)
            {
                sterowanie_lampy(3, 0);
                sterowanie_lampy(1, 0);
                sterowanie_lampy(2, 0);
                sterowanie_lampy(0, 0);
            }
            else
            {
                if (stan_lampa_stojaca == 1)
                {
                    sterowanie_lampy(3, 1);
                    sterowanie_lampy(1, 1);
                    sterowanie_lampy(2, 1);
                    sterowanie_lampy(0, 1);
                }
            }
        }

        /////////////////////////////////////////////// lampa wewnątrz garażu//////////////////////////////////////////////////////////////////////////////////////////
        if (stan_lampa_wewntrzna == 3)
        {
            if (lampa_wiw_gar_4_1 == 1 || lampa_wiw_gar_4_2 == 1)
            {
                sterowanie_lampy(4, 1);
            }

            if (lampa_wiw_gar_4_1 == 0 && lampa_wiw_gar_4_2 == 0)
            {
                sterowanie_lampy(4, 0);
            }
        }
        else
        {
            if (stan_lampa_wewntrzna == 2)
            {
                sterowanie_lampy(4, 0);
            }
            else
            {
                if (stan_lampa_wewntrzna == 1)
                {
                    sterowanie_lampy(4, 1);
                }
            }
        }

        if (noc == 1)
        {
            if (war_raz_czuj_ruchu_1 == 0)
            {
                czujka_ruchu_zasilanie(1); // włączenie zasilania czujników ruchu
                czas_opuz_czujniki_licz = czas;
                war_raz_czuj_ruchu_1 = 1;
                war_raz_czuj_ruchu_2 = 0;
            }
        }
        else
        {
            if (war_raz_czuj_ruchu_2 == 0)
            {
                czujka_ruchu_zasilanie(0); // wyłączenie zasilania czujników ruchu
                war_raz_czuj_ruchu_2 = 1;
                war_raz_czuj_ruchu_1 = 0;
            }
        }

        if (zastrafabram_dol == 1 || zastrafabram_gora == 1)
        {
            zastrafabram = 1;
        }
        if (zastrafabram_dol == 0 && zastrafabram_gora == 0)
        {
            zastrafabram = 0; ////pin załączający zasilanie silników trafa
        }
        digitalWrite(zastrafasilnkibr, zastrafabram); ////pin załączający zasilanie silników trafa
    }
    else
    {
        digitalWrite(zastrafasilnkibr, 0); ////pin załączający zasilanie silników trafa
        war_raz_czuj_ruchu_2 = 0;
        war_raz_czuj_ruchu_1 = 0;
        czujka_ruchu_zasilanie(0);
        sterowanie_lampy(0, 0);
        sterowanie_lampy(1, 0);
        sterowanie_lampy(2, 0);
        sterowanie_lampy(3, 0);
        sterowanie_lampy(4, 0);
    }
    ///////////KONIEC STEROWANIA LAMPAMI/////////////////////////////////////////////////////////////////////////////
    //
    /////////////////////////////////////////////////////////////////////
    // IR remote
    if (IrReceiver.decode())
    {
        if (tryb_ustawien_433 <= 1)
        {
            if (IrReceiver.decodedIRData.decodedRawData == 0xC159B44)
            { // zamknij dolna brama
                komenda_zamknij_dolna_brama();
            }

            if (IrReceiver.decodedIRData.decodedRawData == 0x1F9B44)
            { // otwórz dolna brama
                komenda_otworz_dolna_brama();
            }

            if (IrReceiver.decodedIRData.decodedRawData == 0x17429B44)
            { // zamknij gorna brama
                komenda_zamknij_gorna_brama();
            }

            if (IrReceiver.decodedIRData.decodedRawData == 0x1C569B44)
            { // otwórz gorna brama
                komenda_otworz_gorna_brama();
            }

            if (IrReceiver.decodedIRData.decodedRawData == 0xA1019B44)
            { // lampa wewnetrzna
                czas_wylacz_reczne_lampy_licz = czas;
                stan_lampa_wewntrzna++;

                if (stan_lampa_wewntrzna == 4)
                {
                    stan_lampa_wewntrzna = 1;
                }

                if (stan_lampa_wewntrzna == 1 || stan_lampa_wewntrzna == 2)
                {
                    czaspikaczu1 = czas;
                }
                if (stan_lampa_wewntrzna == 3)
                {
                    czaspikaczu2 = czas;
                }
            }
            if (IrReceiver.decodedIRData.decodedRawData == 0xA2029B44)
            { // lampa stojaca
                czas_wylacz_reczne_lampy_licz = czas;
                stan_lampa_stojaca++;
                if (stan_lampa_stojaca == 4)
                {
                    stan_lampa_stojaca = 1;
                }

                if (stan_lampa_stojaca == 1 || stan_lampa_stojaca == 2)
                {
                    czaspikaczu1 = czas;
                }
                if (stan_lampa_stojaca == 3)
                {
                    czaspikaczu2 = czas;
                }
            }
        }
        IrReceiver.resume();
    }
    //
    // ddfd

    if (mySwitch.available())
    {
        dane_rcswitch = mySwitch.getReceivedValue();
        mySwitch.resetAvailable();

        // 2  tryb programowania
        if (tryb_ustawien_433 >= 2 && tryb_ustawien_433 <= 6)
        {
            czas_pilot_czas_ustawien_licz = czas;
            // tryb_ustawien_433 == 3   otwórz brama dolna
            // tryb_ustawien_433 == 4  zamknij brama dolna
            // tryb_ustawien_433 == 5  otwórz brama górna
            // tryb_ustawien_433 == 6  zamknij brama górna

            // jesli tryb_ustawien_433 == 2  , mruga numer pilota np 0 / pusty
            //  jeśli klikniesz klawisz swiateł to nr pilota ++
            //  jesli kllikniejsz któryś klawisz sterowania bramą to  mruga dół albo góra na przemian z numerem pilota , w tym momęcie zczytywane sa dane z piloita 433MHz  jesli 5 razy zczyta ten sam kod to zapis do eeprom i powrót do mrugania pilotem
            ///     ale jesli klikniesz jakis inny klawisz bramy to anuluje zapis i wyswietla inna kombinacje

            // numer_tabRCswitch_bufor  0 1 2 3 4 5 6 -

            if (numer_tabRCswitch_bufor < 7)
            {
                tabRCswitch_bufor[numer_tabRCswitch_bufor] = dane_rcswitch;
                numer_tabRCswitch_bufor++;

                if (numer_tabRCswitch_bufor >= 7)
                {
                    if (tabRCswitch_bufor[0] == tabRCswitch_bufor[1])
                    {
                        if (tabRCswitch_bufor[0] == tabRCswitch_bufor[2])
                        {
                            if (tabRCswitch_bufor[0] == tabRCswitch_bufor[3])
                            {
                                if (tabRCswitch_bufor[0] == tabRCswitch_bufor[4])
                                {
                                    if (tabRCswitch_bufor[0] == tabRCswitch_bufor[5])
                                    {
                                        if (tabRCswitch_bufor[0] == tabRCswitch_bufor[6])
                                        {                               /// jeśli wszystkie wartości sa takie same to
                                                                        /// zapis do pamięci eeprom //////
                                                                        /// nr_pilota  /// 1,2,3,4,5,6// cyfrowo: 0,16,32,64,128,256
                                            if (tryb_ustawien_433 == 3) // 3 otwórz brama dolna
                                            {
                                                writeLongIntoEEPROM((0 + nr_pilota), tabRCswitch_bufor[0]);
                                            }

                                            if (tryb_ustawien_433 == 4) // 4 zamknij brama dolna
                                            {
                                                writeLongIntoEEPROM((4 + nr_pilota), tabRCswitch_bufor[0]);
                                            }

                                            if (tryb_ustawien_433 == 5) // 5  otwórz brama górna
                                            {
                                                writeLongIntoEEPROM((8 + nr_pilota), tabRCswitch_bufor[0]);
                                            }

                                            if (tryb_ustawien_433 == 6) // 6  zamknij brama górna
                                            {
                                                writeLongIntoEEPROM((12 + nr_pilota), tabRCswitch_bufor[0]);
                                            }
                                            // zajęty eeprom 272 włącznie

                                            numer_tabRCswitch_bufor = 0;
                                            czaspikaczu2 = czas; /// zapinano kod
                                            tryb_ustawien_433 = 2;
                                        }
                                        else
                                        {
                                            numer_tabRCswitch_bufor = 0;
                                            czaspikaczu1 = czas;
                                        }
                                    }
                                    else
                                    {
                                        numer_tabRCswitch_bufor = 0;
                                        czaspikaczu1 = czas;
                                    }
                                }
                                else
                                {
                                    numer_tabRCswitch_bufor = 0;
                                    czaspikaczu1 = czas;
                                }
                            }
                            else
                            {
                                numer_tabRCswitch_bufor = 0;
                                czaspikaczu1 = czas;
                            }
                        }
                        else
                        {
                            numer_tabRCswitch_bufor = 0;
                            czaspikaczu1 = czas;
                        }
                    }
                    else
                    {
                        numer_tabRCswitch_bufor = 0;
                        czaspikaczu1 = czas;
                    }
                }
            }
        }
    }

    if (tryb_ustawien_433 <= 1)
    {
        if (dane_rcswitch == tabRCswitch[0][0] || dane_rcswitch == tabRCswitch[1][0] || dane_rcswitch == tabRCswitch[2][0] || dane_rcswitch == tabRCswitch[3][0] || dane_rcswitch == tabRCswitch[4][0] || dane_rcswitch == tabRCswitch[5][0])
        {
            dane_rcswitch = 0;
            if (czas - czas_pilot1_klik > czas_pilot_klik_dwuklik)
            {
                komenda_otworz_dolna_brama();
            }
            czas_pilot1_klik = czas;
        }

        if (dane_rcswitch == tabRCswitch[0][1] || dane_rcswitch == tabRCswitch[1][1] || dane_rcswitch == tabRCswitch[2][1] || dane_rcswitch == tabRCswitch[3][1] || dane_rcswitch == tabRCswitch[4][1] || dane_rcswitch == tabRCswitch[5][1])
        {
            dane_rcswitch = 0;
            if (czas - czas_pilot2_klik > czas_pilot_klik_dwuklik)
            {
                komenda_zamknij_dolna_brama();
            }

            czas_pilot2_klik = czas;
        }

        if (dane_rcswitch == tabRCswitch[0][2] || dane_rcswitch == tabRCswitch[1][2] || dane_rcswitch == tabRCswitch[2][2] || dane_rcswitch == tabRCswitch[3][2] || dane_rcswitch == tabRCswitch[4][2] || dane_rcswitch == tabRCswitch[5][2])
        {
            dane_rcswitch = 0;
            if (czas - czas_pilot3_klik > czas_pilot_klik_dwuklik)
            {
                komenda_otworz_gorna_brama();
            }
            czas_pilot3_klik = czas;
        }

        if (dane_rcswitch == tabRCswitch[0][3] || dane_rcswitch == tabRCswitch[1][3] || dane_rcswitch == tabRCswitch[2][3] || dane_rcswitch == tabRCswitch[3][3] || dane_rcswitch == tabRCswitch[4][3] || dane_rcswitch == tabRCswitch[5][3])
        {
            dane_rcswitch = 0;
            if (czas - czas_pilot4_klik > czas_pilot_klik_dwuklik)
            {
                komenda_zamknij_gorna_brama();
            }

            czas_pilot4_klik = czas;
        }
    }
    else
    {
        if (czas - czas_pilot_czas_ustawien_licz > czas_pilot_czas_ustawien)
        {
            nr_pilota = 0;
            czaspikaczu2 = czas;
            tryb_ustawien_433 = 0;
            odczyt_Pamieci_EEprom();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////

    /////////// odczyt z nrf24L01/////////////////////////////////////////////////////////////////////
    while (radio.available())
    {
        radio.read(&rxData, sizeof(receivedData));
    }

    if (rxData.numerpilota == 175 || rxData.numerpilota == 129 || rxData.numerpilota == 68 || rxData.numerpilota == 195)
    {
        rxData.numerpilota = 0;

        if (tryb_ustawien_433 <= 1)
        {
            if (rxData.ch1 == 1) //////////otwórz brama dol 1  ///////// kanał  1
            {
                rxData.ch1 = 0;
                komenda_otworz_dolna_brama();
            }
            if (rxData.ch2 == 1) //////////zamknij brama dol 1 ///////// kanał  2
            {
                rxData.ch2 = 0;
                komenda_zamknij_dolna_brama();
            }
            if (rxData.ch3 == 1) //////////otwórz brama  góra 2   ///////// kanał  3
            {
                rxData.ch3 = 0;
                komenda_otworz_gorna_brama();
            }
            if (rxData.ch4 == 1) //////////zamknij 2 brama góra   ///////// kanał  4
            {
                rxData.ch4 = 0;
                komenda_zamknij_gorna_brama();
            }
        }
        else
        {
            rxData.ch1 = 0;
            rxData.ch2 = 0;
            rxData.ch3 = 0;
            rxData.ch4 = 0;
        }
    }
    if (rxData.numerpilota == 200)
    {
        rxData.numerpilota = 0;

        if (rxData.ch1 == 1)
        { //////////czujnik dom nie łapie  ///////// kanał  1
            rxData.ch1 = 0;
            czujnik_dom = 0;
            // czaspikaczu2 = czas;
        }

        if (rxData.ch2 == 1)
        { //////////czujnik dom  łapie  ///////// kanał  1
            rxData.ch2 = 0;
            czujnik_dom = 1;
            //   czaspikaczu1 = czas;
        }

        // puste kanały z czujnika
        if (rxData.ch3 == 1)
        {
            rxData.ch3 = 0;
        }
        if (rxData.ch4 == 1)
        {
            rxData.ch4 = 0;
        }
    }

    //

    ///////////////////////////DOLNA BRAMA//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////sterowanie
    if (aktybrama1 == 1)
    {
        //////////////////////////// przyciski/////////+/-20j////////////////////////////////////////////////////////////////////////////////////////////////
        if (przyciski_brama_dolna == 1)
        {
            adcprzycbrdolna = analogRead(przycbragardolna); // wejscie przycisku wewnetrznego sterowanie brama dolna 2
                                                            /* if (monitoring == 1)
                                                             {
                                                                 Serial.print("adc przycisk dolny : ");
                                                                 Serial.print(adcprzycbrdolna);
                                                                  Serial.println(" ");
                                                             }
                                                             */

            /// odczytane 510///
            if (490 < adcprzycbrdolna && adcprzycbrdolna < 530) // zamykanie bramy dolnej 510
            {
                if (czas - czas_filtr_przyc_dolna2 > czasnacisniencia_przyc && warprzycbrdul == 0) // jesli przez ten czas naciśniemy przycisk to zadziała (filtr pików napięcia na wejściu adc)
                {
                    warprzycbrdul = 2;
                    czasprzyczamykaniedolna = czas;
                }
            }
            else
            {
                czas_filtr_przyc_dolna2 = czas;
            }

            if (warprzycbrdul == 2)
            {
                if (tryb_ustawien_433 <= 1)
                {
                    if (czas - czasprzyczamykaniedolna < czasprzycdlugie)
                    {
                        if (adcprzycbrdolna < 30)
                        {
                            warprzycbrdul = 0;
                            komenda_zamknij_dolna_brama();
                        }
                    }
                    else
                    {
                        if (bladRR_R == 0 && bladRR == 0 && bladR == 0 && bladB == 0 && bladD == 0 && bladF == 0 && bladJ == 0 && bladK == 0 && bladM == 0 && brama1 == 0 && brama11 == 0 && otworzdolnazopuznienie == 0 && bladG == 0 && bladL == 0)
                        {
                            if (otworzgorazopuznienie == 1)
                            {
                                czasprzyczamykaniegoraanimacja = czas;
                                animacjawolnegora = 10;
                            }
                            czasprzyczamykaniedolnaanimacja = czas;
                            animacjawolnedolna = 10;
                            otworzdolnazopuznienie = 1;
                            czaspikaczu2 = czas;
                        }
                        else
                        {
                            komenda_zamknij_dolna_brama();
                        }
                        warprzycbrdul = 3;
                    }
                }
                else
                {
                    if (adcprzycbrdolna < 30)
                    {
                        czas_pilot_czas_ustawien_licz = czas;
                        tryb_ustawien_433 = 4;
                        warprzycbrdul = 0;
                    }
                }
            }

            ///// otwieranie  jesli klawisz krutko naciśniety jesli długo to ustawienia pilotów
            if (297 < adcprzycbrdolna && adcprzycbrdolna < 337)
            {
                if (czas - czas_filtr_przyc_dolna1 > czasnacisniencia_przyc && warprzycbrdul == 0)
                {

                    warprzycbrdul = 1;
                    czasprzycotwieraniedolna = czas;
                }
            }
            else
            {
                czas_filtr_przyc_dolna1 = czas;
            }

            if (warprzycbrdul == 1)
            {
                if (tryb_ustawien_433 <= 1)
                {
                    if (czas - czasprzycotwieraniedolna < czas_przyc_dlugie_prog)
                    {
                        if (adcprzycbrdolna < 30)
                        {
                            warprzycbrdul = 0;
                            komenda_otworz_dolna_brama();
                        }
                    }
                    else
                    {
                        if (blad == 0)
                        {
                            if (tryb_ustawien_433 == 0)
                            {
                                tryb_ustawien_433 = 1;
                            }
                            else if (tryb_ustawien_433 == 1)
                            {
                                tryb_ustawien_433 = 2;
                                czas_pilot_czas_ustawien_licz = czas;
                                czaspikaczu2 = czas;
                            }
                        }
                        warprzycbrdul = 3;
                    }
                }
                else
                {
                    if (adcprzycbrdolna < 30)
                    {
                        czas_pilot_czas_ustawien_licz = czas;
                        tryb_ustawien_433 = 3;
                        warprzycbrdul = 0;
                    }
                }
            }
            ///// otwieranie  jesli klawisz krutko naciśniety jesli długo to ustawienia pilotów/////

            // powrót stanów początkowych
            if (adcprzycbrdolna < 30 && warprzycbrdul == 3)
            {
                warprzycbrdul = 0;
                if (tryb_ustawien_433 == 1)
                {
                    tryb_ustawien_433 = 0;
                }
            }

            if (otworzdolnazopuznienie == 1)
            {
                if (czas - czasprzyczamykaniedolnaanimacja > czsaopuszczeniezwloka)
                {
                    czaspikaczu1 = czas;
                    if (blad == 0)
                    {
                        if (otworzgorazopuznienie == 0)
                        {
                            pcf8574(cyfrybledow[animacjawolnedolna]);
                        }
                        else
                        {
                            if (animacjawolnedolna == 10 || animacjawolnedolna == 8 || animacjawolnedolna == 6 || animacjawolnedolna == 4 || animacjawolnedolna == 2 || animacjawolnedolna == 0)
                            {
                                pcf8574(cyfrybledow[animacjawolnedolna]);
                            }
                        }
                    }

                    czasprzyczamykaniedolnaanimacja = czas;
                    if (animacjawolnedolna == 0)
                    {

                        czaspikaczu2 = czas;
                        otworzdolnazopuznienie = 0;
                        bramadolnazamykanie();
                    }
                    animacjawolnedolna--;
                }
            }
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        }

        /// zabezpieczenie szyny przy odłączonej bramie//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (drugieczujki_szyny_dolna == 1 && czas - czasopuznwlczujdol1 > czasopuznwlczujdol2)
        {
            czujka_dolna_szyna = digitalRead(switch_dolna_szyna);                     // silnik: dotyka == 1  // niedotyka == 0
                                                                                      // otwieranie
            if (czujka_dolna_szyna == 1 && brama1 == 1 && var_stanu_szyna_dolna == 0) // jesli sie otwiera i dojedzie do konca szyny
            {
                if (pwmwsilnkdol >= 1010 || tryb_szynadolna_1 == 0)
                {
                    var_stanu_szyna_dolna = 1;

                    brama1 = 0;
                    pwmwsilnkdol = 0;
                    bladR = 33; /// bład przy otwieraniu jak czujnik do jedzie do górnejego switcha ,brama dolna
                    blad = 1;
                    czas_wys_bl_sw_dolny = czas;

                    if (noc == 1 && wlacz_l_wew_polczasu == 1 && rodzaj_zasilania == 0)
                    { /// wyącz lampe wewnątrz podczas błędu
                        lampa_wiw_gar_4_1 = 0;
                    }

                    if (tryb_szynadolna_1 == 1) /// //podrót ///
                    {
                        czas_wylna_srodku_dol = czas; // czas pomocniczy

                        if (tryb_szynadolna_11 == 0) ///  powolny
                        {
                            waropuzdolotw = 1;
                            czasopuzniwylprzekdolotw = czas;
                        }
                        else /// szybki
                        {
                            czasmaxzamudul1 = czas;
                            czasminpradsilnikdolny = czas;
                            brama11 = 1;
                            kieruneksilnikdol1(0);
                            kieruneksilnikdol2(1);
                            czasopuznwlczujdol1 = czas; /// opuxnienie za załączenie czujników
                        }
                    }
                    else // // stop
                    {
                        zasczujniki_dolnbr(0);
                        if (tryb_szynadolna_11 == 0) ///  powolne
                        {
                            waropuzdolotw = 1;
                            czasopuzniwylprzekdolotw = czas;
                        }
                        else //// ostry
                        {
                            kieruneksilnikdol1(0);
                            wyl_zas_dol();
                        }
                    }
                }
                else
                {
                    off_forewer_dolna_czujniki_szyny();
                }
            }

            // zamykanie  //jak czujnik do jedzie do górnejego switcha ,brama dolna
            if (czujka_dolna_szyna == 1 && brama11 == 1 && var_stanu_szyna_dolna == 0)
            {
                if (pwmwsilnkdol >= 1010 || tryb_szynadolna_1 == 0)
                {
                    var_stanu_szyna_dolna = 1;
                    brama11 = 0;
                    pwmwsilnkdol = 0;
                    bladRR = 35; /// bład przy otwieraniu jak czujnik do jedzie do górnejego switcha ,brama dolna
                    blad = 1;
                    czas_wys_bl_sw_dolny = czas;

                    if (noc == 1 && wlacz_l_wew_polczasu == 1 && rodzaj_zasilania == 0)
                    { /// wyącz lampe wewnątrz podczas błędu
                        lampa_wiw_gar_4_1 = 0;
                    }

                    if (tryb_szynadolna_1 == 1) // podrót
                    {
                        czas_wylna_srodku_dol = czas; // czas pomocniczy szyny

                        if (tryb_szynadolna_11 == 0) // powolne
                        {
                            waropuzdolzam = 1;
                            czasopuzniwylprzekdolzam = czas;
                        }
                        else // ostre
                        {
                            czasmaxotwudul1 = czas;
                            czasminpradsilnikdolny = czas;
                            brama1 = 1;
                            kieruneksilnikdol1(1);
                            kieruneksilnikdol2(0);
                            czasopuznwlczujdol1 = czas; /// opuxnienie za załączenie czujników
                        }
                    }
                    else /// == 0 // stop
                    {
                        zasczujniki_dolnbr(0);
                        if (tryb_szynadolna_11 == 0) // powolne
                        {
                            waropuzdolzam = 1;
                            czasopuzniwylprzekdolzam = czas;
                        }
                        else ////tryb_szynadolna_11 == 1 ostre
                        {
                            kieruneksilnikdol1(0);
                            wyl_zas_dol();
                        }
                    }
                }
                else
                {
                    off_forewer_dolna_czujniki_szyny();
                }
            }

            if (czujka_dolna_szyna == 0) ////////////////////////dopisać błedy jakie moze to zatrzymac  oprucz
            {
                if (var_stanu_szyna_dolna == 1)
                {
                    var_stanu_szyna_dolna = 0;
                }
            }

            if (bladRR == 35 && bladR == 33)
            {
                off_forewer_dolna_czujniki_szyny();
            }

            /////////////// zatrzymanie w połowie czasów "max" zawracania po bledzie switcha po otwieraniu
            if (tryb_szynadolna_1 == 1)
            {
                if (bladR == 33 && brama11 == 1)
                {
                    if (czas - czas_wylna_srodku_dol > czas_powrotu_po_bledzieszyny)
                    {
                        if (czujka_dolna_szyna == 0)
                        {
                            bladR = 0;
                            // powolne wyłączenie
                            if (noc == 1 && wlacz_l_wew_polczasu == 1)
                            { /// włącz lampe wewnątrz w połowie czasu otwierania
                                lampa_wiw_gar_4_1 = 0;
                            }
                            pwmwsilnkdol = 0;
                            brama11 = 0;
                            zasczujniki_dolnbr(0);
                            waropuzdolzam = 1;
                            czasopuzniwylprzekdolzam = czas;
                        }
                        else
                        {
                            off_forewer_dolna_czujniki_szyny();
                        }
                    }
                }

                if (bladRR == 35 && brama1 == 1)
                {
                    if (czas - czas_wylna_srodku_dol > czas_powrotu_po_bledzieszyny)
                    {
                        if (czujka_dolna_szyna == 0)
                        {
                            bladRR = 0;
                            // powolne wyłączenie
                            if (noc == 1 && wlacz_l_wew_polczasu == 1)
                            { /// włącz lampe wewnątrz w połowie czasu otwierania
                                lampa_wiw_gar_4_1 = 0;
                            }
                            pwmwsilnkdol = 0;
                            brama1 = 0;
                            zasczujniki_dolnbr(0);
                            waropuzdolotw = 1;
                            czasopuzniwylprzekdolotw = czas;
                        }
                        else
                        {
                            off_forewer_dolna_czujniki_szyny();
                        }
                    }
                }
            }
        }
        // koniec czujek szyny
        //////////////////////
        /////otwieranie bramy 1
        if (brama1 == 1)
        {
            if ((zmien_czuj_zas == 0 && rodzaj_zasilania == 0) || (zmien_czuj_zas == 0 && rodzaj_zasilania == 1 && pwmwsilnkgor == 0))
            {
                if (czas - czasopuznwlczujdol1 > czasopuznwlczujdol2) /// opuxnienie za załączenie czujników
                {
                    if (digitalRead(wejsciehaLL11) == 0 || bladR == 33 || bladRR == 35)
                    { /////czujnik otwarcia jesli 0 niedolega ,jesli 1 brama dolega
                        if (czas - czaspwmplussilnikdolna > czas_pwm_wlacz && pwmwsilnkdol < 1023)
                        {
                            pwmwsilnkdol++;
                            czaspwmplussilnikdolna = czas;
                        }
                        if (czas - czasminpradsilnikdolny > czaswykrniskpradusildul && pradsilnikbrdolna < minpradsilnikdolny && czujnik_pradu_dolna_brama == 1)
                        {
                            blad = 1;
                            bladM = 30;
                            pwmwsilnkdol = 0;
                            zasczujniki_dolnbr(0);
                            kieruneksilnikdol1(0); //////przekaznik prawo brama górna
                            brama1 = 0;
                            czaswysbladpraddul = czas;
                            wyl_zas_dol();
                        }
                    }
                    else
                    {
                        brama1 = 0;
                        pwmwsilnkdol = 0;
                        if (noc == 1 && wlacz_l_wew_polczasu == 1 && rodzaj_zasilania == 0)
                        { /// włącz lampe wewnątrz w połowie czasu otwierania
                            lampa_wiw_gar_4_1 = 1;
                        }

                        if (digitalRead(wejsciehaLL12) == 0) //////jesli halle sa sprawne
                        {
                            if (trybopuzdolotwhall == 0) // ostre wyłaczenie
                            {
                                wyl_zas_dol();
                                kieruneksilnikdol1(0);
                            }
                            else // powolne opuxnienie
                            {
                                waropuzdolotw = 1;
                                czasopuzniwylprzekdolotw = czas;
                            }
                        }
                        else
                        { //////bład jesli halle sa niesprawdne
                            kieruneksilnikdol1(0);
                            bladB = 18;
                            blad = 1;
                            czaswysblhalldolny = czas;
                            wyl_zas_dol();
                        }
                        zasczujniki_dolnbr(0);
                        czaspikaczu1 = czas;
                    }
                }
                else
                {
                    pwmwsilnkdol = 0;
                    czasmaxotwudul1 = czas;
                    czasminpradsilnikdolny = czas;
                    czas_wylna_srodku_dol = czas; // czas pomocniczy szyny
                }
            }
            else
            {
                pwmwsilnkdol = 0;
                czasmaxotwudul1 = czas;
                czasminpradsilnikdolny = czas;
                czas_wylna_srodku_dol = czas; // czas pomocniczy szyny
                czasopuznwlczujdol1 = czas;   /// opuxnienie za załączenie czujników
            }

            if (czas - czasmaxotwudul1 > czasmaxotwudul2)
            { /////bład za długiego otwierania brama dolna
                bladJ = 1;
                blad = 1;
                pwmwsilnkdol = 0;
                zasczujniki_dolnbr(0);
                kieruneksilnikdol1(0); //////przekaznik prawo brama górna
                brama1 = 0;
                czaswysbladNtwdul = czas;
                wyl_zas_dol();
            }

            if (czas - czasmaxotwudul1 > czasmaxotwudul2 / 2 && noc == 1 && wlacz_l_wew_polczasu == 1 && rodzaj_zasilania == 0)
            { /// włącz lampe wewnątrz w połowie czasu otwierania
                lampa_wiw_gar_4_1 = 1;
            }
        }

        /////////zamyklanie bramy 1////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (brama11 == 1)
        {

            if ((zmien_czuj_zas == 0 && rodzaj_zasilania == 0) || (zmien_czuj_zas == 0 && rodzaj_zasilania == 1 && pwmwsilnkgor == 0))
            {
                if (czas - czasopuznwlczujdol1 > czasopuznwlczujdol2) /// opuxnienie za załączenie czujników
                {

                    if (digitalRead(wejsciehaLL12) == 0 || bladR == 33 || bladRR == 35)
                    {

                        if (digitalRead(wejsciefoto1dol) == 0 || bladR == 33 || bladRR == 35)
                        {

                            if (czas - czaspwmplussilnikdolna > czas_pwm_wlacz && pwmwsilnkdol < 1023)
                            {
                                pwmwsilnkdol++;
                                czaspwmplussilnikdolna = czas;
                            }

                            if (czas - czasminpradsilnikdolny > czaswykrniskpradusildul && pradsilnikbrdolna < minpradsilnikdolny && czujnik_pradu_dolna_brama == 1) // minimalny prad
                            {
                                blad = 1;
                                bladM = 30;
                                pwmwsilnkdol = 0;
                                zasczujniki_dolnbr(0);
                                kieruneksilnikdol2(0); //////przekaznik prawo brama górna
                                brama11 = 0;
                                czaswysbladpraddul = czas;
                                wyl_zas_dol();
                            }
                        }
                        else
                        {
                            bladD = 16;
                            blad = 1;
                            brama11 = 0;
                            czaswysblfotodolny = czas;
                            pwmwsilnkdol = 0;
                            if (trybfotodolna == 1) /// jesli 1 wyłącza wsztyko
                            {
                                zasczujniki_dolnbr(0);
                                if (trybopuzdolzamfoto == 0) // ostre wyłaczenie
                                {
                                    wyl_zas_dol();
                                    kieruneksilnikdol2(0);
                                }
                                else // powolne wyłaczenie
                                {
                                    waropuzdolzam = 1;
                                    czasopuzniwylprzekdolzam = czas;
                                }
                            }
                            else
                            { /// jesli 0 ruch z powrotem
                                if (trybopuzdolzamfoto == 0)
                                {
                                    czasmaxotwudul1 = czas;
                                    czasminpradsilnikdolny = czas;
                                    kieruneksilnikdol1(1);
                                    kieruneksilnikdol2(0);
                                    brama1 = 1;
                                    czasopuznwlczujdol1 = czas;
                                }
                                else
                                {
                                    waropuzdolzam = 1;
                                    czasopuzniwylprzekdolzam = czas;
                                }
                            }
                        }
                    }
                    else
                    {
                        if (noc == 1 && wlacz_l_wew_polczasu == 1)
                        { /// włącz lampe wewnątrz w połowie czasu otwierania
                            lampa_wiw_gar_4_1 = 0;
                        }
                        pwmwsilnkdol = 0;
                        brama11 = 0;

                        if (digitalRead(wejsciehaLL11) == 0) // wyłancza róch jesli saprawny hall
                        {
                            if (trybopuzdolzamhall == 0)
                            {
                                wyl_zas_dol();
                                kieruneksilnikdol2(0); //////przekaznik lewo brama górna
                            }
                            else
                            {
                                waropuzdolzam = 1;
                                czasopuzniwylprzekdolzam = czas;
                            }
                        }
                        else // bład jesli nie sprawny hall
                        {
                            kieruneksilnikdol2(0); //////przekaznik lewo brama górna
                            bladB = 18;
                            blad = 1;
                            czaswysblhalldolny = czas;
                            wyl_zas_dol();
                        }
                        zasczujniki_dolnbr(0);
                        czaspikaczu1 = czas;
                    }
                }
                else
                {
                    pwmwsilnkdol = 0;
                    czasmaxzamudul1 = czas;
                    czasminpradsilnikdolny = czas;
                    czas_wylna_srodku_dol = czas; // czas pomocniczy szyny
                }
            }
            else
            {
                pwmwsilnkdol = 0;
                czasmaxzamudul1 = czas;
                czasminpradsilnikdolny = czas;
                czas_wylna_srodku_dol = czas; // czas pomocniczy szyny
                czasopuznwlczujdol1 = czas;   /// opuxnienie za załączenie czujników
            }

            if (czas - czasmaxzamudul1 > czasmaxzamudul2)
            { /////bład za długiego zamykania brama dolna
                bladK = 27;
                blad = 1;
                pwmwsilnkdol = 0;
                zasczujniki_dolnbr(0);
                kieruneksilnikdol2(0);
                brama11 = 0;
                czaswysbladzamdol = czas;
                wyl_zas_dol();
            }

            if (czas - czasmaxzamudul1 > czasmaxzamudul2 / 2 && noc == 1 && wlacz_l_wew_polczasu == 1)
            { /// wyłącz lampe wewnątrz w połowie czasu otwierania
                lampa_wiw_gar_4_1 = 0;
            }
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // opuźnienia przekaźników brama dolna

        if (waropuzdolotw == 1 && czas - czasopuzniwylprzekdolotw > zmczasopuzniwylprzekdolotw)
        {
            waropuzdolotw = 0;
            kieruneksilnikdol1(0);
            if ((bladpradpodjdolna11 == 0 && bladF == 20 && trybzabezpamper1 == 0) || (tryb_szynadolna_1 == 1 && tryb_szynadolna_11 == 0 && bladR == 33))
            {
                if (bladB == 0 && bladG == 0 && bladL == 0 && bladD == 0)
                {
                    kieruneksilnikdol2(1);
                    czasmaxzamudul1 = czas;
                    czasminpradsilnikdolny = czas;
                    brama11 = 1;
                    czasopuznwlczujdol1 = czas;
                }
            }
            wyl_zas_dol();
        }

        if (waropuzdolzam == 1 && czas - czasopuzniwylprzekdolzam > zmczasopuzniwylprzekdolzam)
        {
            waropuzdolzam = 0;
            kieruneksilnikdol2(0);
            if ((bladpradpodjdolna1 == 0 && bladF == 20 && trybzabezpamper1 == 0) || (bladD == 16 && trybfotodolna == 0 && trybopuzdolzamfoto == 1) || (tryb_szynadolna_1 == 1 && tryb_szynadolna_11 == 0 && bladRR == 35))
            {
                if (bladB == 0 && bladG == 0 && bladL == 0)
                {
                    kieruneksilnikdol1(1);
                    czasmaxotwudul1 = czas;
                    czasminpradsilnikdolny = czas;
                    brama1 = 1;
                    czasopuznwlczujdol1 = czas;
                }
            }
            wyl_zas_dol();
        }

        /////////////////////////////////ZABEZPIECZENIA ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ////ampery brama dolna
        if (czujnik_pradu_dolna_brama == 1)
        {
            if (czas - czaspomiarampdolna >= czas_co_ile_liczyprad)
            {
                if (liczpomar1 <= 600)
                {
                    pomiar_adc_prad_dolny = analogRead(amperilnikdolnab);
                    prad_dolny_odczyt_raz = ((((pomiar_adc_prad_dolny * 5.0) / 1023.0) - 2.5) / 0.100); // odczyt pradu

                    if (prad_dolny_odczyt_raz < 0)
                    { // zabezpieczenie przed umieny prądem i błedem pomiaru
                        prad_dolny_odczyt_raz = prad_dolny_odczyt_raz * (-1);
                    }

                    pradsilnikbrdolnachwila = pradsilnikbrdolnachwila + prad_dolny_odczyt_raz;
                    liczpomar1++;
                }
                else
                {
                    pradsilnikbrdolna = (pradsilnikbrdolnachwila / 600.0);
                    pradsilnikbrdolnachwila = 0;
                    liczpomar1 = 0;

                    if (monitoring == 1 && brama11 == 1)
                    {
                        Serial.print("Prad br. dolna zamykania : ");
                        Serial.print(pradsilnikbrdolna);
                        Serial.print("A");
                        Serial.println(" ");
                    }

                    if (monitoring == 1 && brama1 == 1)
                    {
                        Serial.print("Prad br. dolna otwiernia : ");
                        Serial.print(pradsilnikbrdolna);
                        Serial.print("A");
                        Serial.println(" ");
                    }
                }
                czaspomiarampdolna = czas;
            }

            ////jesli zaduzy prad silnika brama dolna otwieraniem
            if (pradsilnikbrdolna > maxpradsillnikbrdolnaotw)
            {
                if (czas - czas_zwloki_brdolna_otworz_prad_licz > czas_zwloki_brdolna_prad && brama1 == 1 && bladpradpodjdolna11 == 0)
                {
                    czasbladpraddolny = czas;
                    bladF = 20;
                    blad = 1;
                    pwmwsilnkdol = 0;
                    brama1 = 0;
                    bladpradpodjdolna1 = 1;
                    if (trybzabezpamper1 == 1 || bladR == 33 || bladRR == 35) // wyłącza wszystko
                    {
                        zasczujniki_dolnbr(0);

                        if (trybopuzdolotwprad == 0)
                        {
                            wyl_zas_dol();
                            kieruneksilnikdol1(0); //////przekaznik prawo brama górna
                        }
                        else
                        {
                            waropuzdolotw = 1;
                            czasopuzniwylprzekdolotw = czas;
                        }
                    }
                    else /// odwraca ruch
                    {
                        if (trybopuzdolotwprad == 0) // odwraca odrazu
                        {
                            czasmaxzamudul1 = czas;
                            czasminpradsilnikdolny = czas;
                            brama11 = 1;
                            kieruneksilnikdol1(0);
                            kieruneksilnikdol2(1);
                            czasopuznwlczujdol1 = czas;
                        }
                        else /// odwraca ruch z opuxnieniem
                        {
                            waropuzdolotw = 1;
                            czasopuzniwylprzekdolotw = czas;
                        }
                    }
                }
            }
            else
            {
                czas_zwloki_brdolna_otworz_prad_licz = czas;
            }

            ////////////////////////
            ////jesli zaduzy prad silnika brama dolna zamykanie
            if (pradsilnikbrdolna > maxpradsillnikbrdolnazam)
            {
                if (czas - czas_zwloki_brdolna_zamknij_prad_licz > czas_zwloki_brdolna_prad && brama11 == 1 && bladpradpodjdolna1 == 0)
                {
                    czasbladpraddolny = czas;
                    bladF = 20;
                    blad = 1;
                    pwmwsilnkdol = 0;
                    brama11 = 0;
                    bladpradpodjdolna11 = 1;
                    if (trybzabezpamper1 == 1 || bladR == 33 || bladRR == 35) // stop
                    {
                        zasczujniki_dolnbr(0);
                        if (trybopuzdolzamprad == 0)
                        {
                            wyl_zas_dol();
                            kieruneksilnikdol2(0); //////przekaznik lewo brama górna
                        }
                        else
                        {
                            waropuzdolzam = 1;
                            czasopuzniwylprzekdolzam = czas;
                        }
                    }
                    else // powrót
                    {
                        if (trybopuzdolzamprad == 0)
                        {
                            czasminpradsilnikdolny = czas;
                            czasmaxotwudul1 = czas;
                            brama1 = 1;
                            kieruneksilnikdol1(1);
                            kieruneksilnikdol2(0);
                            czasopuznwlczujdol1 = czas;
                        }
                        else
                        {
                            waropuzdolzam = 1;
                            czasopuzniwylprzekdolzam = czas;
                        }
                    }
                }
            }
            else
            {
                czas_zwloki_brdolna_zamknij_prad_licz = czas;
            }
        }

        if (bladF == 20 && czas - czasbladpraddolny > czaswyswietlaniabledow)
        { // silnik bramy dolnej
            bladF = 0;
            bladpradpodjdolna11 = 0;
            bladpradpodjdolna1 = 0;
        }
        if (bladM == 30 && czas - czaswysbladpraddul > czaswyswietlaniabledow)
        {
            bladM = 0;
        }
        if (bladJ == 1 && czas - czaswysbladNtwdul > czaswyswietlaniabledow)
        {
            bladJ = 0;
        }
        if (bladK == 27 && czas - czaswysbladzamdol > czaswyswietlaniabledow)
        {
            bladK = 0;
        }
        if (czas - czaswysblhalldolny > czaswyswietlaniabledow && bladB == 18)
        { //// długosc wyswietlania bledu czujnika hala bramy dolnej
            bladB = 0;
        }
        if (czas - czaswysblfotodolny > czaswyswietlaniabledow && bladD == 16)
        { //// długosc wyswietlania bledu fotokomorki bramy dolnej
            bladD = 0;
        }
        if (czas - czas_wys_bl_sw_dolny > (czasmaxotwudul2 + czasmaxzamudul2))
        {
            if (bladR == 33)
            {
                var_stanu_szyna_dolna = 0;
                bladR = 0;
            }

            if (bladRR == 35)
            {
                var_stanu_szyna_dolna = 0;
                bladRR = 0;
            }

            if (bladRR_R == 9)
            {
                bladRR_R = 0;
            }
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////BRAMA 2/////////GÓRNA//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// brama górna
    if (aktybrama2 == 1)
    {
        // przyciski
        // tolerancja +-20j

        //  0, 614 = 2 wcisniete , 2 niewcisniete
        if (przyciski_brama_gorna == 1)
        {
            adcprzycbrgorna = analogRead(przycbragargorna); // wejscie przycisku wewnetrznego sterowanie brama gorna 1
                                                            /*
                                                                        if (monitoring == 1)
                                                                        {
                                                                            Serial.print("adc przycisk gorny : ");
                                                                            Serial.print(adcprzycbrgorna);
                                                                            Serial.println(" ");
                                                                        }*/
                                                            // odczytane 512  // 512// zamykanie bramy gornej
            if (492 < adcprzycbrgorna && adcprzycbrgorna < 532)
            {
                if (czas - czas_filtr_przyc_gorna2 > czasnacisniencia_przyc && warprzycbrgora == 0)
                {

                    warprzycbrgora = 2;
                    czasprzyczamykaniegora = czas;
                }
            }
            else
            {
                czas_filtr_przyc_gorna2 = czas;
            }

            if (warprzycbrgora == 2)
            {
                if (tryb_ustawien_433 <= 1)
                {
                    if (czas - czasprzyczamykaniegora < czasprzycdlugie)
                    {
                        if (adcprzycbrgorna < 30)
                        {
                            warprzycbrgora = 0;
                            komenda_zamknij_gorna_brama();
                        }
                    }
                    else
                    {
                        if (bladS == 0 && bladSS == 0 && bladA == 0 && bladC == 0 && bladE == 0 && bladH == 0 && bladI == 0 && bladN == 0 && brama22 == 0 && brama2 == 0 && otworzgorazopuznienie == 0 && bladG == 0 && bladL == 0 && bladSS_S == 0)
                        {

                            if (otworzdolnazopuznienie == 1)
                            {
                                animacjawolnedolna = 10;
                                czasprzyczamykaniedolnaanimacja = czas;
                            }
                            czasprzyczamykaniegoraanimacja = czas;
                            animacjawolnegora = 10;
                            otworzgorazopuznienie = 1;
                            czaspikaczu2 = czas;
                        }
                        else
                        {
                            komenda_zamknij_gorna_brama();
                        }
                        warprzycbrgora = 3;
                    }
                }
                else
                {
                    if (adcprzycbrgorna < 30)
                    {
                        czas_pilot_czas_ustawien_licz = czas;
                        tryb_ustawien_433 = 6;
                        warprzycbrgora = 0;
                    }
                }
            }

            // /// otwieranie gorna brama 316 ODCZYTANE!!!!!
            if (296 < adcprzycbrgorna && adcprzycbrgorna < 336) // 341 /// otwieranie bramy gornej
            {
                if (czas - czas_filtr_przyc_gorna1 > czasnacisniencia_przyc && warprzycbrgora == 0)
                {

                    warprzycbrgora = 1;
                    czasprzycotwieraniegora = czas;
                }
            }
            else
            {
                czas_filtr_przyc_gorna1 = czas;
            }

            if (warprzycbrgora == 1)
            {
                if (tryb_ustawien_433 <= 1)
                {
                    if (czas - czasprzycotwieraniegora < czas_przyc_dlugie_prog)
                    {
                        if (adcprzycbrgorna < 30)
                        {
                            warprzycbrgora = 0;
                            komenda_otworz_gorna_brama();
                        }
                    }
                    else
                    {
                        if (blad == 0)
                        {
                            if (tryb_ustawien_433 == 0)
                            {
                                tryb_ustawien_433 = 1;
                            }
                            else if (tryb_ustawien_433 == 1)
                            {
                                czas_pilot_czas_ustawien_licz = czas;
                                tryb_ustawien_433 = 2;
                                czaspikaczu2 = czas;
                            }
                        }
                        warprzycbrgora = 3;
                    }
                }
                else
                {
                    if (adcprzycbrgorna < 30)
                    {
                        czas_pilot_czas_ustawien_licz = czas;
                        tryb_ustawien_433 = 5;
                        warprzycbrgora = 0;
                    }
                }
            }

            if (adcprzycbrgorna < 30 && warprzycbrgora == 3)
            {
                warprzycbrgora = 0;
                if (tryb_ustawien_433 == 1)
                {
                    tryb_ustawien_433 = 0;
                }
            }

            if (otworzgorazopuznienie == 1) ///////////OPUŹNIENIE PRZYCISKU
            {
                if (czas - czasprzyczamykaniegoraanimacja > czsaopuszczeniezwloka)
                {
                    czaspikaczu1 = czas;
                    if (blad == 0)
                    {

                        if (otworzdolnazopuznienie == 0)
                        {

                            pcf8574(cyfrybledow[animacjawolnegora] & 0b01111111);
                        }
                        else
                        {

                            if (animacjawolnegora == 9 || animacjawolnegora == 7 || animacjawolnegora == 5 || animacjawolnegora == 3 || animacjawolnegora == 1)
                            {
                                pcf8574(cyfrybledow[animacjawolnegora] & 0b01111111);
                            }
                        }
                    }
                    czasprzyczamykaniegoraanimacja = czas;
                    if (animacjawolnegora == 0)
                    {
                        czaspikaczu2 = czas;
                        otworzgorazopuznienie = 0;
                        bramagornazamykanie();
                    }
                    animacjawolnegora--;
                }
            }
        }

        /// otwieranie bramy gornej

        /// zabezpieczenie szyny przy odłączonej bramie//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (drugieczujki_szyny_gorna == 1 && czas - czasopuznwlczujgora1 > czasopuznwlczujgora2)
        {
            czujka_gorna_szyna = digitalRead(switch_gorna_szyna);                     // silnik: dotyka == 1  // niedotyka == 0 styk nc
                                                                                      // otwieranie
            if (czujka_gorna_szyna == 1 && brama2 == 1 && var_stanu_szyna_gorna == 0) // jesli sie otwiera i dojedzie do konca szyny
            {
                if (pwmwsilnkgor >= 1010 || tryb_szynagorna_1 == 0)
                {
                    var_stanu_szyna_gorna = 1;
                    brama2 = 0;
                    pwmwsilnkgor = 0;
                    bladS = 34; /// bład przy otwieraniu jak czujnik do jedzie do górnejego switcha ,brama dolna
                    blad = 1;
                    czas_wys_bl_sw_gorny = czas;

                    if (noc == 1 && wlacz_l_wew_polczasu == 1 && rodzaj_zasilania == 0)
                    { /// wyącz lampe wewnątrz podczas błędu
                        lampa_wiw_gar_4_2 = 0;
                    }

                    if (tryb_szynagorna_1 == 1) /// == 1 //podrót ///
                    {
                        czas_wylna_srodku_gor = czas; // czas pomocniczy

                        if (tryb_szynagorna_11 == 0) ///  == 0 // powolne
                        {
                            waropuzgorotw = 1;
                            czasopuzniwylprzekgorotw = czas;
                        }
                        else //// == 1 ostre
                        {
                            czasmaxzamugora1 = czas;
                            czasminpradsilnikgorny = czas;
                            brama22 = 1;
                            kieruneksilnikgor1(0);
                            kieruneksilnikgor2(1);
                            czasopuznwlczujgora1 = czas; /// opużnienie za załączenie czujników
                        }
                    }
                    else /// == 0 // stop
                    {
                        zasczujniki_gorabr(0);
                        if (tryb_szynagorna_11 == 0) /// tryb_szynadolna_11 == 0 // powolne
                        {
                            waropuzgorotw = 1;
                            czasopuzniwylprzekgorotw = czas;
                        }
                        else ////tryb_szynadolna_11 == 1 ostre
                        {
                            kieruneksilnikgor1(0);
                            wyl_zas_gora();
                        }
                    }
                }
                else
                {
                    off_forewer_gorna_czujniki_szyny();
                }
            }
            // zamykanie
            if (czujka_gorna_szyna == 1 && brama22 == 1 && var_stanu_szyna_gorna == 0)
            {
                if (pwmwsilnkgor >= 1010 || tryb_szynagorna_1 == 0)
                {
                    var_stanu_szyna_gorna = 1;
                    brama22 = 0;
                    pwmwsilnkgor = 0;
                    bladSS = 36; /// bład przy otwieraniu jak czujnik do jedzie do górnejego switcha ,brama dolna
                    blad = 1;
                    czas_wys_bl_sw_gorny = czas;

                    if (noc == 1 && wlacz_l_wew_polczasu == 1 && rodzaj_zasilania == 0)
                    { /// wyącz lampe wewnątrz podczas błędu
                        lampa_wiw_gar_4_2 = 0;
                    }

                    if (tryb_szynagorna_1 == 1) /// == 1 //podrót ///
                    {
                        czas_wylna_srodku_gor = czas; // czas pomocniczy szyny

                        if (tryb_szynagorna_11 == 0) ///== 0 // powolne
                        {
                            waropuzgorzam = 1;
                            czasopuzniwylprzekgorzam = czas;
                        }
                        else ///== 1 ostre
                        {
                            czasmaxotwugora1 = czas;
                            czasminpradsilnikgorny = czas;
                            brama2 = 1;
                            kieruneksilnikgor1(1);
                            kieruneksilnikgor2(0);
                            czasopuznwlczujgora1 = czas; /// opużnienie za nałączenie czujników
                        }
                    }
                    else ///== 0 // stop
                    {
                        zasczujniki_gorabr(0);
                        if (tryb_szynagorna_11 == 0) // 1 == 0 // powolne
                        {
                            waropuzgorzam = 1;
                            czasopuzniwylprzekgorzam = czas;
                        }
                        else /// == 1 ostre
                        {
                            kieruneksilnikgor1(0);
                            wyl_zas_gora();
                        }
                    }
                }
                else
                {
                    off_forewer_gorna_czujniki_szyny();
                }
            }

            if (czujka_gorna_szyna == 0) ////////////////////////dopisać błedy jakie moze to zatrzymac  oprucz
            {
                if (var_stanu_szyna_gorna == 1)
                {
                    var_stanu_szyna_gorna = 0;
                }
            }

            if (bladSS == 36 && bladS == 34)
            {
                off_forewer_gorna_czujniki_szyny();
            }

            if (tryb_szynagorna_1 == 1) // zatrzymanie w połowie czasów "max" zawracania po bledzie switcha po otwieraniu
            {
                if (bladS == 34 && brama22 == 1)
                {
                    if (czas - czas_wylna_srodku_gor > czas_powrotu_po_bledzieszyny)
                    {
                        if (czujka_gorna_szyna == 0)
                        {
                            bladS = 0;
                            // powolne wyłączenie
                            if (noc == 1 && wlacz_l_wew_polczasu == 1)
                            { /// włącz lampe wewnątrz w połowie czasu otwierania
                                lampa_wiw_gar_4_2 = 0;
                            }
                            pwmwsilnkgor = 0;
                            brama22 = 0;
                            zasczujniki_gorabr(0);
                            waropuzgorzam = 1;
                            czasopuzniwylprzekgorzam = czas;
                        }
                        else
                        {
                            off_forewer_gorna_czujniki_szyny();
                        }
                    }
                }

                if (bladSS == 36 && brama2 == 1)
                {
                    if (czas - czas_wylna_srodku_gor > czas_powrotu_po_bledzieszyny)
                    {

                        if (czujka_gorna_szyna == 0)
                        {
                            bladSS = 0;
                            // powolne wyłączenie
                            if (noc == 1 && wlacz_l_wew_polczasu == 1)
                            { /// włącz lampe wewnątrz w połowie czasu otwierania
                                lampa_wiw_gar_4_2 = 0;
                            }
                            pwmwsilnkgor = 0;
                            brama2 = 0;
                            zasczujniki_gorabr(0);
                            waropuzgorotw = 1;
                            czasopuzniwylprzekgorotw = czas;
                        }
                        else
                        {
                            off_forewer_gorna_czujniki_szyny();
                        }
                    }
                }
            }
        }

        /////otwieranie bramy górnej//////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (brama2 == 1)
        {
            if ((zmien_czuj_zas == 0 && rodzaj_zasilania == 0) || (zmien_czuj_zas == 0 && rodzaj_zasilania == 1 && pwmwsilnkdol == 0))
            {

                if (czas - czasopuznwlczujgora1 > czasopuznwlczujgora2) /// opużnienie za nałączenie czujników
                {
                    if (digitalRead(wejsciehaLL21) == 0 || bladS == 34 || bladSS == 36)
                    {
                        if (czas - czaspwmplussilnikgora > czas_pwm_wlacz && pwmwsilnkgor < 1023) //////341
                        {
                            pwmwsilnkgor++;
                            czaspwmplussilnikgora = czas;
                        }

                        if (czas - czasminpradsilnikgorny > czaswykrniskpradusilgora && pradsilnikbrgorna < minpradsilnikgorny && czujnik_pradu_gorna_brama == 1)
                        {
                            blad = 1;
                            bladN = 29;
                            pwmwsilnkgor = 0;
                            zasczujniki_gorabr(0);
                            kieruneksilnikgor1(0); //////przekaznik prawo brama górna
                            brama2 = 0;
                            czaswysbladpradgora = czas;
                            // wyl_zas_1();
                            wyl_zas_gora();
                        }
                    }
                    else
                    {
                        brama2 = 0;
                        pwmwsilnkgor = 0;

                        if (noc == 1 && wlacz_l_wew_polczasu == 1 && rodzaj_zasilania == 0)
                        { /// włącz lampe wewnątrz w połowie czasu otwierania
                            lampa_wiw_gar_4_2 = 1;
                        }

                        if (digitalRead(wejsciehaLL22) == 0) /// 2  hal sprawny
                        {
                            if (trybopuzgorotwhall == 0) // ostre
                            {
                                wyl_zas_gora();
                                kieruneksilnikgor1(0); //////przekaznik prawo brama górna
                            }
                            else // powolne
                            {
                                waropuzgorotw = 1;
                                czasopuzniwylprzekgorotw = czas;
                            }
                        }
                        else // 2 hall uszkodzony
                        {
                            kieruneksilnikgor1(0); //////przekaznik prawo brama górna
                            bladA = 19;
                            blad = 1;
                            czaswysblhallgorny = czas;
                            wyl_zas_gora();
                        }
                        zasczujniki_gorabr(0);
                        czaspikaczu1 = czas;
                    }
                }
                else
                {
                    pwmwsilnkgor = 0;
                    czasmaxotwugora1 = czas;
                    czasminpradsilnikgorny = czas;
                    czas_wylna_srodku_gor = czas; // czas pomocniczy szyny
                }
            }
            else
            {
                pwmwsilnkgor = 0;
                czasmaxotwugora1 = czas;
                czasminpradsilnikgorny = czas;
                czas_wylna_srodku_gor = czas; // czas pomocniczy szyny
                czasopuznwlczujgora1 = czas;  /// opużnienie za nałączenie czujników
            }

            /////////////////////////////
            // bład czasu bramy górnej
            if (czas - czasmaxotwugora1 > czasmaxotwugora2)
            { /////bład za długiego otwierania brama gorna
                bladH = 26;
                blad = 1;
                pwmwsilnkgor = 0;
                zasczujniki_gorabr(0);
                kieruneksilnikgor1(0); //////przekaznik prawo brama górna
                brama2 = 0;
                czaswysbladNtwgora = czas;
                //  wyl_zas_1();
                wyl_zas_gora();
            }

            if (czas - czasmaxotwugora1 > czasmaxotwugora2 / 2 && noc == 1 && wlacz_l_wew_polczasu == 1 && rodzaj_zasilania == 0)
            {
                lampa_wiw_gar_4_2 = 1;
            }
        }

        //////////zamyklanie bramy górnej///////////////////////////////////////////////////////////////////////////////////////////////
        if (brama22 == 1)
        {
            if ((zmien_czuj_zas == 0 && rodzaj_zasilania == 0) || (zmien_czuj_zas == 0 && rodzaj_zasilania == 1 && pwmwsilnkdol == 0))
            {
                if (czas - czasopuznwlczujgora1 > czasopuznwlczujgora2) /// opużnienie za nałączenie czujników
                {

                    if (digitalRead(wejsciehaLL22) == 0 || bladS == 34 || bladSS == 36)
                    {
                        if (digitalRead(wejsciefoto1gor) == 0 || bladS == 34 || bladSS == 36)
                        {

                            if (czas - czaspwmplussilnikgora > czas_pwm_wlacz && pwmwsilnkgor < 1023) /// 1023
                            {
                                pwmwsilnkgor++;
                                czaspwmplussilnikgora = czas;
                            }
                            if (czas - czasminpradsilnikgorny > czaswykrniskpradusilgora && pradsilnikbrgorna < minpradsilnikgorny && czujnik_pradu_gorna_brama == 1) // bład zaniskiego pradu
                            {
                                blad = 1;
                                bladN = 29;
                                pwmwsilnkgor = 0;
                                zasczujniki_gorabr(0);
                                kieruneksilnikgor2(0); //////przekaznik prawo brama górna
                                brama22 = 0;
                                czaswysbladpradgora = czas;
                                wyl_zas_gora();
                            }
                        }
                        else
                        {
                            bladC = 17;
                            blad = 1;
                            brama22 = 0;
                            czaswysblfotogorny = czas;
                            pwmwsilnkgor = 0;
                            if (trybfotogorna == 1) // wyłacza wszytko
                            {
                                zasczujniki_gorabr(0);
                                if (trybopuzgorzamfoto == 0) /// wyłacza bez opuźniena
                                {
                                    wyl_zas_gora();
                                    kieruneksilnikgor2(0);
                                }
                                else /// wyłacza z opuźniem
                                {
                                    waropuzgorzam = 1;
                                    czasopuzniwylprzekgorzam = czas;
                                }
                            }
                            else // wraca z powrotem
                            {
                                if (trybopuzgorzamfoto == 0) // powrót bez opuznienia
                                {
                                    czasmaxotwugora1 = czas;
                                    czasminpradsilnikgorny = czas;
                                    kieruneksilnikgor1(1);
                                    kieruneksilnikgor2(0);
                                    brama2 = 1;
                                    czasopuznwlczujgora1 = czas; /// opużnienie za nałączenie czujników
                                }
                                else /// powrót po opuznienu
                                {
                                    waropuzgorzam = 1;
                                    czasopuzniwylprzekgorzam = czas;
                                }
                            }
                        }
                    }
                    else
                    {
                        if (noc == 1 && wlacz_l_wew_polczasu == 1)
                        { /// włącz lampe wewnątrz w połowie czasu otwierania
                            lampa_wiw_gar_4_2 = 0;
                        }
                        pwmwsilnkgor = 0;
                        brama22 = 0;
                        if (digitalRead(wejsciehaLL21) == 0) ////jesli 2 hall jest sprawny
                        {
                            if (trybopuzgorzamhall == 0) /// wyłacza bez opuźnienia
                            {
                                wyl_zas_gora();
                                kieruneksilnikgor2(0); //////przekaznik lewo brama górna
                            }
                            else
                            {
                                waropuzgorzam = 1;
                                czasopuzniwylprzekgorzam = czas;
                            }
                        }
                        else /// jesli drugi hal jest uszkodzony
                        {
                            kieruneksilnikgor2(0); //////przekaznik lewo brama górna
                            bladA = 19;
                            blad = 1;
                            czaswysblhallgorny = czas;
                            wyl_zas_gora();
                        }
                        zasczujniki_gorabr(0);
                        czaspikaczu1 = czas;
                    }
                }
                else
                {
                    pwmwsilnkgor = 0;
                    czasmaxzamugora1 = czas;
                    czasminpradsilnikgorny = czas;
                    czas_wylna_srodku_gor = czas; // czas pomocniczy szyny
                }
            }
            else
            {
                pwmwsilnkgor = 0;
                czasmaxzamugora1 = czas;
                czasminpradsilnikgorny = czas;
                czas_wylna_srodku_gor = czas; // czas pomocniczy szyny
                czasopuznwlczujgora1 = czas;  /// opużnienie za nałączenie czujników
            }

            if (czas - czasmaxzamugora1 > czasmaxzamugora2)
            { /////bład za długiego zamykania brama gorna
                bladI = 28;
                blad = 1;
                pwmwsilnkgor = 0;
                zasczujniki_gorabr(0);
                kieruneksilnikgor2(0); //////przekaznik lewo brama górna
                brama22 = 0;
                czaswysbladzamgora = czas;
                wyl_zas_gora();
            }
            if (czas - czasmaxzamugora1 > czasmaxzamugora2 / 2 && noc == 1 && wlacz_l_wew_polczasu == 1)
            {
                lampa_wiw_gar_4_2 = 0;
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (waropuzgorotw == 1 && czas - czasopuzniwylprzekgorotw > zmczasopuzniwylprzekgorotw)
        {
            waropuzgorotw = 0;
            kieruneksilnikgor1(0);
            if (bladpradpodjgorna11 == 0 && bladE == 21 && trybzabezpamper2 == 0)
            {
                if ((bladA == 0 && bladG == 0 && bladL == 0 && bladC == 0) || (tryb_szynagorna_1 == 1 && tryb_szynagorna_11 == 0 && bladS == 34))
                {
                    kieruneksilnikgor2(1);
                    czasmaxzamugora1 = czas;
                    czasminpradsilnikgorny = czas;
                    brama22 = 1;
                    czasopuznwlczujgora1 = czas; /// opużnienie za nałączenie czujników
                }
            }
            wyl_zas_gora();
        }

        if (waropuzgorzam == 1 && czas - czasopuzniwylprzekgorzam > zmczasopuzniwylprzekgorzam)
        {

            waropuzgorzam = 0;
            kieruneksilnikgor2(0);
            if ((bladpradpodjgorna1 == 0 && bladE == 21 && trybzabezpamper2 == 0) || (bladC == 17 && trybfotogorna == 0 && trybopuzgorzamfoto == 1) || (tryb_szynagorna_1 == 1 && tryb_szynagorna_11 == 0 && bladSS == 36))
            {
                if (bladA == 0 && bladG == 0 && bladL == 0)
                {
                    kieruneksilnikgor1(1);
                    czasmaxotwugora1 = czas;
                    czasminpradsilnikgorny = czas;
                    brama2 = 1;
                    czasopuznwlczujgora1 = czas; /// opużnienie za nałączenie czujników
                }
            }
            wyl_zas_gora();
        }

        ////ampery brama gorna ((20A -> czułośc 100mV na 1A
        //////zlicznie błedu ampera

        if (czujnik_pradu_gorna_brama == 1)
        {
            if (czas - czaspomiarampgorna >= czas_co_ile_liczyprad)
            {
                if (liczpomar2 <= 600)
                {
                    pomiar_adc_prad_gorny = analogRead(amperilnikgornab); // wejscie czujnik pradu silnika bramy górnej 2  ////zabezpieczenie pradowe silnik 1 brama górna
                    prad_gorny_odczyt_raz = ((((pomiar_adc_prad_gorny * 5.0) / 1023.0) - 2.5) / 0.100);

                    if (prad_gorny_odczyt_raz > 0)
                    { // zabezpieczenie przed umieny prądem i błedem pomiaru
                        prad_gorny_odczyt_raz = prad_gorny_odczyt_raz * (-1);
                    }

                    pradsilnikbrgornachwila = pradsilnikbrgornachwila + prad_gorny_odczyt_raz;
                    liczpomar2++;
                }
                else
                {

                    pradsilnikbrgornachwila = pradsilnikbrgornachwila * (-1); /// razy -1 bo czujnik jest podłaczony odwrotnie i pokazje ujemny prad
                    pradsilnikbrgorna = (pradsilnikbrgornachwila / 600.0);
                    pradsilnikbrgornachwila = 0;
                    liczpomar2 = 0;

                    if (monitoring == 1 && brama2 == 1)
                    {
                        Serial.print("Prad br. górna otwierania: ");
                        Serial.print(pradsilnikbrgorna);
                        Serial.print("A");
                        Serial.println(" ");
                    }
                    if (monitoring == 1 && brama22 == 1)
                    {
                        Serial.print("Prad br. górna zamykania: ");
                        Serial.print(pradsilnikbrgorna);
                        Serial.print("A");
                        Serial.println(" ");
                    }
                }
                czaspomiarampgorna = czas;
            }
            //
            //
            // otwieranie
            /// jesli zaduzy prad silnika brama gorna przy otwieraniu
            if (pradsilnikbrgorna > maxpradsillnikbrgornaotw)
            {
                if (czas - czas_zwloki_brgorna_otworz_prad_licz > czas_zwloki_brgorna_prad && brama2 == 1 && bladpradpodjgorna11 == 0)
                {
                    czasbladpradgorny = czas; //
                    bladE = 21;               //
                    blad = 1;                 //
                    pwmwsilnkgor = 0;         //
                    brama2 = 0;
                    bladpradpodjgorna1 = 1;

                    if (trybzabezpamper2 == 1 || bladS == 34 || bladSS == 36) // wyłacza wszytko
                    {
                        zasczujniki_gorabr(0);
                        if (trybopuzgorotwprad == 0) // ostre
                        {
                            wyl_zas_gora();
                            kieruneksilnikgor1(0); //////przekaznik prawo brama górna
                        }
                        else
                        {
                            waropuzgorotw = 1;
                            czasopuzniwylprzekgorotw = czas;
                        }
                    }
                    else // odwraca ruch
                    {
                        if (trybopuzgorotwprad == 0)
                        {
                            czasmaxzamugora1 = czas;
                            czasminpradsilnikgorny = czas;
                            brama22 = 1;
                            kieruneksilnikgor1(0);
                            kieruneksilnikgor2(1);
                            czasopuznwlczujgora1 = czas; /// opużnienie za nałączenie czujników
                        }
                        else
                        {
                            waropuzgorotw = 1;
                            czasopuzniwylprzekgorotw = czas;
                        }
                    }
                }
            }
            else
            {
                czas_zwloki_brgorna_otworz_prad_licz = czas;
            }

            ///
            /// zamykanie
            ////jesli zaduzy prad silnika brama gorna przy zamykaniu
            if (pradsilnikbrgorna > maxpradsillnikbrgornazam)
            {
                if (czas - czas_zwloki_brgorna_zamknij_prad_licz > czas_zwloki_brgorna_prad && brama22 == 1 && bladpradpodjgorna1 == 0)
                {
                    czasbladpradgorny = czas;
                    bladE = 21;
                    blad = 1;
                    pwmwsilnkgor = 0;
                    brama22 = 0;
                    bladpradpodjgorna11 = 1;

                    if (trybzabezpamper2 == 1 || bladS == 34 || bladSS == 36) /// wyłacza wszystko
                    {
                        zasczujniki_gorabr(0);
                        if (trybopuzgorzamprad == 0)
                        {
                            wyl_zas_gora();
                            kieruneksilnikgor2(0); //////przekaznik lewo brama górna
                        }
                        else
                        {
                            waropuzgorzam = 1;
                            czasopuzniwylprzekgorzam = czas;
                        }
                    }
                    else /// odwraca ruch
                    {
                        if (trybopuzgorzamprad == 0)
                        {
                            czasmaxotwugora1 = czas;
                            czasminpradsilnikgorny = czas;
                            brama2 = 1;
                            kieruneksilnikgor1(1);
                            kieruneksilnikgor2(0);
                            czasopuznwlczujgora1 = czas; /// opużnienie za nałączenie czujników
                        }
                        else
                        {
                            waropuzgorzam = 1;
                            czasopuzniwylprzekgorzam = czas;
                        }
                    }
                }
            }
            else
            {
                czas_zwloki_brgorna_zamknij_prad_licz = czas;
            }
        }

        if (bladE == 21 && czas - czasbladpradgorny > czaswyswietlaniabledow)
        { // silnik bramy górnej
            bladE = 0;
            bladpradpodjgorna11 = 0;
            bladpradpodjgorna1 = 0;
        }
        if (bladN == 29 && czas - czaswysbladpradgora > czaswyswietlaniabledow)
        {
            bladN = 0;
        }
        if (bladH == 26 && czas - czaswysbladNtwgora > czaswyswietlaniabledow)
        {
            bladH = 0;
        }
        if (bladI == 28 && czas - czaswysbladzamgora > czaswyswietlaniabledow)
        {
            bladI = 0;
        }
        if (czas - czaswysblhallgorny > czaswyswietlaniabledow && bladA == 19)
        { //// długosc wyswietlania bledu czujnika hala bramy górnej
            bladA = 0;
        }
        if (czas - czaswysblfotogorny > czaswyswietlaniabledow && bladC == 17)
        { //// długosc wyswietlania bledu fotokomorki bramy gornej
            bladC = 0;
        }
        if (czas - czas_wys_bl_sw_gorny > (czasmaxzamugora2 + czasmaxotwugora2))
        { // wyłączenie błedu jesli nie ma powrotu/ zabezpiecezenie szyny
          //  if (tryb_szynagorna_1 == 0)
          // {
            if (bladS == 34)
            {
                var_stanu_szyna_gorna = 0;
                bladS = 0;
            }
            if (bladSS == 36)
            {
                var_stanu_szyna_gorna = 0;
                bladSS = 0;
            }
            // }

            if (bladSS_S == 37)
            {
                bladSS_S = 0;
            }
        }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // wspulne
    if (bladL == 25 && czas - czaswysbladwlzasil > czaswyswietlaniabledow)
    {
        bladL = 0;
    }
    if (bladG == 24 && czas - czaswysbladwylzasil > czaswyswietlaniabledow)
    {
        bladG = 0;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////ANIMACJE WYSWIETLACZA 7 SEGMETOWEGO///USTAWIENIA PILOTÓW 433MHz //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (blad == 0 && tryb_ustawien_433 >= 2)
    {
        // tryb_ustawien_433 == 2 tryb programowania mruga numer pilota
        //  tryb_ustawien_433 == 3   otwórz brama dolna
        //  tryb_ustawien_433 == 4  zamknij brama dolna
        //  tryb_ustawien_433 == 5  otwórz brama górna
        //  tryb_ustawien_433 == 6  zamknij brama górna

        // int nr_pilota = 0;    /// 1,2,3,4,5,6//  0,16,32,64,128,256

        if (czas - czasanimacji_pilot433 > 700)
        {
            animacja_pilot_k = !animacja_pilot_k;
            czasanimacji_pilot433 = czas;
        }

        if (animacja_pilot_k == 1) // wyswietla numer pilota
        {

            if (tryb_ustawien_433 == 3 || tryb_ustawien_433 == 5)
            {
                a = 0;
            }

            if (tryb_ustawien_433 == 4 || tryb_ustawien_433 == 6)
            {
                a = 9;
            }

            if (nr_pilota == 0)
            {
                pcf8574(cyfrybledow[1]); /// pilot "0"
            }
            else if (nr_pilota == 16)
            {
                pcf8574(cyfrybledow[2]); /// pilot "1"
            }
            else if (nr_pilota == 32)
            {
                pcf8574(cyfrybledow[3]); /// pilot "2"
            }
            else if (nr_pilota == 64)
            {
                pcf8574(cyfrybledow[4]); /// pilot "3"
            }
            else if (nr_pilota == 128)
            {
                pcf8574(cyfrybledow[5]); /// pilot "4"
            }
            else if (nr_pilota == 256)
            {
                pcf8574(cyfrybledow[6]); /// pilot "5"
            }
        }
        else /// wyświetla ruch bramy jaki zostanie zaprogramowany
        {
            if (tryb_ustawien_433 == 2)
            {
                pcf8574(cyfrybledow[0]); /// pusty
            }

            if (tryb_ustawien_433 == 3)
            { //  otwórz brama dolna

                if (czas - czasanimacji > 60)
                {
                    pcf8574(animacjalewy[a]);
                    a++;
                    if (a > 9)
                    {
                        a = 0;
                    }
                    czasanimacji = czas;
                }
            }

            if (tryb_ustawien_433 == 4)
            { //  zamknij brama dolna
                if (czas - czasanimacji > 60)
                {
                    pcf8574(animacjalewy[a]);
                    a--;
                    if (a < 0)
                    {
                        a = 9;
                    }
                    czasanimacji = czas;
                }
            }

            if (tryb_ustawien_433 == 5)
            { //  otwórz brama górna
                if (czas - czasanimacji > 60)
                {
                    pcf8574(animacjaprawy[a]);
                    a++;
                    if (a > 9)
                    {
                        a = 0;
                    }
                    czasanimacji = czas;
                }
            }

            if (tryb_ustawien_433 == 6)
            { //  zamknij brama górna
                if (czas - czasanimacji > 60)
                {
                    pcf8574(animacjaprawy[a]);
                    a--;
                    if (a < 0)
                    {
                        a = 9;
                    }
                    czasanimacji = czas;
                }
            }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////ANIMACJE WYSWIETLACZA 7 SEGMETOWEGO//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (blad == 0 && otworzdolnazopuznienie == 0 && otworzgorazopuznienie == 0 && tryb_ustawien_433 <= 1)
    {

        if (brama1 == 1 && brama11 == 0 && brama2 == 0 && brama22 == 0)
        { /// DOLNA OTWIERANIE   GÓRNA STOP
            if (czas - czasanimacji > 300)
            {
                pcf8574(animacjalewy[a]);
                a++;
                if (a > 9)
                {
                    a = 0;
                }
                czasanimacji = czas;
            }
        }
        if (brama1 == 0 && brama11 == 1 && brama2 == 0 && brama22 == 0)
        { /// DOLNA ZAMYKANIE    GÓRNA STOP
            if (czas - czasanimacji > 300)
            {
                pcf8574(animacjalewy[a]);
                a--;
                if (a < 0)
                {
                    a = 9;
                }
                czasanimacji = czas;
            }
        }

        if (brama1 == 0 && brama11 == 0 && brama2 == 1 && brama22 == 0)
        { /// GÓRNA OTWIERANIE    DOLNA STOP
            if (czas - czasanimacji > 300)
            {
                pcf8574(animacjaprawy[a]);
                a++;
                if (a > 9)
                {
                    a = 0;
                }
                czasanimacji = czas;
            }
        }
        if (brama1 == 0 && brama11 == 0 && brama2 == 0 && brama22 == 1)
        { /// GÓRNA ZAMYKANIE    DOLNA STOP
            if (czas - czasanimacji > 300)
            {
                pcf8574(animacjaprawy[a]);
                a--;
                if (a < 0)
                {
                    a = 9;
                }
                czasanimacji = czas;
            }
        }
        if (brama1 == 1 && brama11 == 0 && brama2 == 1 && brama22 == 0)
        { /// DOLNA OTWIERANIE   GÓRNA OTWIERANIE
            if (czas - czasanimacji > 300)
            {
                pcf8574(animacjaprawy[a] & animacjalewy[a]);
                a++;
                if (a > 9)
                {
                    a = 0;
                }
                czasanimacji = czas;
            }
        }
        if (brama1 == 0 && brama11 == 1 && brama2 == 0 && brama22 == 1)
        { /// GÓRNA ZAMYKANIE    DOLNA ZAMYKANIE
            if (czas - czasanimacji > 300)
            {
                pcf8574(animacjaprawy[a] & animacjalewy[a]);
                a--;
                if (a < 0)
                {
                    a = 9;
                }
                czasanimacji = czas;
            }
        }
        if (brama1 == 1 && brama11 == 0 && brama2 == 0 && brama22 == 1)
        { /// GÓRNA ZAMYKANIE    DOLNA OTWIERANIE

            if (czas - czasanimacji > 300)
            {
                pcf8574(animacjaobaprzeciwnie[a]);
                a++;
                if (a > 9)
                {
                    a = 0;
                }
                czasanimacji = czas;
            }
        }
        if (brama1 == 0 && brama11 == 1 && brama2 == 1 && brama22 == 0)
        { /// GÓRNA OTWIERANIE    DOLNA ZAMYKANIE
            if (czas - czasanimacji > 300)
            {
                pcf8574(animacjaobaprzeciwnie[a]);
                a--;
                if (a < 0)
                {
                    a = 9;
                }
                czasanimacji = czas;
            }
        }

        if (brama1 == 0 && brama11 == 0 && brama2 == 0 && brama22 == 0 && b == 1)
        {
            pcf8574(cyfrybledow[0]);
            a = 9;
            aaa = 5;
            b = 0;
        }

        if ((brama1 == 1 && b == 0) || (brama11 == 1 && b == 0) || (brama2 == 1 && b == 0) || (brama22 == 1 && b == 0))
        {
            b = 1;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////bład nie  pojawienia sie zasilania mimo  załalczenia//////////////////////////////////////
    if (zmien_czuj_zas == 1 && zastrafabram == 1)
    { // bład nie pojawienia sie zasilania mimo  załalczenia//////////////////////////////////////
        if (czas - czaswlaczeniazass1 > czaswlaczeniazasilania1)
        {
            bladL = 25;
            blad = 1;
            czaswysbladwlzasil = czas;
            if (warwylwszystbrakzas == 1)
            {
                pwmwsilnkgor = 0;
                pwmwsilnkdol = 0;
                zasczujniki_dolnbr(0);
                zasczujniki_gorabr(0);
                kieruneksilnikgor2(0); //////przekaznik lewo brama górna
                kieruneksilnikgor1(0); //////przekaznik prawo brama górna
                kieruneksilnikdol2(0); //////przekaznik lewo brama górna
                kieruneksilnikdol1(0); //////przekaznik prawo brama górna
                brama1 = 0;
                brama11 = 0;
                brama2 = 0;
                brama22 = 0;
                warwylwszystbrakzas = 0;
                zastrafabram_dol = 0;
                zastrafabram_gora = 0;
                wyl_zas_gora();
                wyl_zas_dol();
            }
        }
    }

    if (zastrafabram == 0)
    {
        czaswlaczeniazass1 = czas;
        warwylwszystbrakzas = 1;
    }

    ///////////////////////////bład  pojawienia sie zasilania mimo braku załalczenia//////////////////////////////////////
    if (rodzaj_zasilania == 0) /// 0 siec / 1 akumulator
    {

        if (zmien_czuj_zas == 0 && zastrafabram == 0)
        { /// jesli pojawi sie zasilanie po wyłączeniu trafa , i bedzie dalej po skonczeniu opuźnienia

            if (zmienna_przelaczenia == 1)
            {
                czas_przelaczenia = czas;
                zmienna_przelaczenia = 2;
            }

            if (czas - czaswylaczeniazass1 > czaswylaczeniazasilania1 && czas - czas_przelaczenia > czas_przelaczenia_zasilania)
            {
                bladG = 24;
                blad = 1;
                czaswysbladwylzasil = czas;
            }
        }
        else
        {
            zmienna_przelaczenia = 1;
        }
        if (zastrafabram == 1)
        {
            czaswylaczeniazass1 = czas;
        }
    }
    else
    {
        czaswylaczeniazass1 = czas;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////piny wyjsciowe ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    OCR3B = pwmwsilnkgor;
    OCR3C = pwmwsilnkdol;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////wyswietlanie bledow po koleji ///////////////////////////////////////////////////

    if (blad == 1)
    {
        tryb_ustawien_433 = 0;
        nr_pilota = 0;

        if (czas - czasbledu > (unsigned long)550)
        {
            if (aa == 1)
            {
                aa = 2;
            }
            if (aa == 0)
            {
                if (stopp == 1)
                {
                    if (bladA > 0)
                    {
                        pcf8574(cyfrybledow[bladA]); //////////////////////////błąd czujnika halla gorna brama      H.
                        stopp = 2;
                    }
                    else
                    {
                        stopp = 3;
                    }
                }
                if (stopp == 3)
                {
                    if (bladB > 0)
                    {
                        pcf8574(cyfrybledow[bladB]); //////////////////////////błąd czujnika halla  dolna  brama      H
                        stopp = 4;
                    }
                    else
                    {
                        stopp = 5;
                    }
                }
                if (stopp == 5)
                {
                    if (bladC > 0)
                    {
                        pcf8574(cyfrybledow[bladC]); //////////////////////////błąd fotokomórki górna  brama      F.
                        stopp = 6;
                    }
                    else
                    {
                        stopp = 7;
                    }
                }
                if (stopp == 7)
                {
                    if (bladD > 0)
                    {
                        pcf8574(cyfrybledow[bladD]); //////////////////////////błąd fotokomórki dolna  brama      F
                        stopp = 8;
                    }
                    else
                    {
                        stopp = 9;
                    }
                }
                if (stopp == 9)
                {
                    if (bladE > 0)
                    {
                        pcf8574(cyfrybledow[bladE]); //////////////////////////bład zaduzego poboru pradu silnik  gorna  brama      P.
                        stopp = 10;
                    }
                    else
                    {
                        stopp = 11;
                    }
                }
                if (stopp == 11)
                {
                    if (bladF > 0)
                    {
                        pcf8574(cyfrybledow[bladF]); //////////////////////////bład zaduzego poboru pradu silnik  dolna  brama      P
                        stopp = 12;
                    }
                    else
                    {
                        stopp = 13;
                    }
                }
                if (stopp == 13)
                {
                    if (bladG > 0)
                    {
                        pcf8574(cyfrybledow[bladG]); ////////////////////////bład przy  pojawieniu sie zasilania mimo braku załalczenia   U
                        stopp = 14;
                    }
                    else
                    {
                        stopp = 15;
                    }
                }
                if (stopp == 15)
                {
                    if (bladH > 0)
                    {
                        pcf8574(cyfrybledow[bladH]); ////////////////////////bład zadługiego otwierania bramy gornej   O.  26
                        stopp = 16;
                    }
                    else
                    {
                        stopp = 17;
                    }
                }
                if (stopp == 17)
                {
                    if (bladI > 0)
                    {
                        pcf8574(cyfrybledow[bladI]); ////////////////////////bład zadługiego zamykania bramy gornej   o. 28
                        stopp = 18;
                    }
                    else
                    {
                        stopp = 19;
                    }
                }
                if (stopp == 19)
                {
                    if (bladJ > 0)
                    {
                        pcf8574(cyfrybledow[bladJ]); ////////////////////////bład zadługiego otwierania bramy dolnej   0 1
                        stopp = 20;
                    }
                    else
                    {
                        stopp = 21;
                    }
                }
                if (stopp == 21)
                {
                    if (bladK > 0)
                    {
                        pcf8574(cyfrybledow[bladK]); ////////////////////////bład zadługiego zamykania bramy dolnej   o  27
                        stopp = 22;
                    }
                    else
                    {
                        stopp = 23;
                    }
                }
                if (stopp == 23)
                {
                    if (bladL > 0)
                    {
                        pcf8574(cyfrybledow[bladL]); ////////////////////////bład niepojawienia sie napiecia silników mimo załączenia |^|   (odwrucone U) 25
                        stopp = 24;
                    }
                    else
                    {
                        stopp = 25;
                    }
                }
                if (stopp == 25)
                {
                    if (bladM > 0)
                    {
                        pcf8574(cyfrybledow[bladM]); ////////////////////////bład zaniskiego poboru silnik dolny  S  30
                        stopp = 26;
                    }
                    else
                    {
                        stopp = 27;
                    }
                }
                if (stopp == 27)
                {
                    if (bladN > 0)
                    {
                        pcf8574(cyfrybledow[bladN]); ////////////////////////bład zaniskiego poboru silnik gorny    S. 29
                        stopp = 28;
                    }
                    else
                    {
                        stopp = 29;
                    }
                }
                if (stopp == 29)
                {
                    if (bladR > 0)
                    {
                        pcf8574(cyfrybledow[bladR]); ////////////////////////bład załączenia krancowki szyny przy otwieraniu Dolna brama // 3 pionowe  33
                        stopp = 30;
                    }
                    else
                    {
                        stopp = 31;
                    }
                }
                if (stopp == 31)
                {
                    if (bladRR > 0)
                    {
                        pcf8574(cyfrybledow[bladRR]); ////////////////////////bład załączenia krancowki szyny przy zamykaniu Dolna brama //2 poziome  35
                        stopp = 32;
                    }
                    else
                    {
                        stopp = 33;
                    }
                }
                if (stopp == 33)
                {
                    if (bladS > 0)
                    {
                        pcf8574(cyfrybledow[bladS]); ////////////////////////bład załączenia krancowki szyny przy otwieraniu Górna brama // 3 pionowe...  34
                        stopp = 34;
                    }
                    else
                    {
                        stopp = 35;
                    }
                }
                if (stopp == 35)
                {
                    if (bladSS > 0)
                    {
                        pcf8574(cyfrybledow[bladSS]); ////////////////////////bład załączenia krancowki szyny przy zamykaniu Górna brama // 2 poziome...  36
                        stopp = 36;
                    }
                    else
                    {
                        stopp = 37;
                    }
                }
                if (stopp == 37)
                {
                    if (bladSS_S > 0)
                    {
                        pcf8574(cyfrybledow[bladSS_S]); ////////////////////////bład zablokowania swicza jeśli silnik zjedzie lub nie puści 37 //   8.
                        stopp = 38;
                    }
                    else
                    {
                        stopp = 39;
                    }
                }
                if (stopp == 39)
                {
                    if (bladRR_R > 0)
                    {
                        pcf8574(cyfrybledow[bladRR_R]); ////////////////////////bład zablokowania swicza jeśli silnik zjedzie lub nie puści 9 //   8
                        stopp = 40;
                    }
                    else
                    {
                        stopp = 41;
                    }
                }

                // tu dopisywać błedy  ////tu dopisywać błedy  ////tu dopisywać błedy

                if (stopp == 41)
                {
                    pcf8574(0b11111110); /// led srodkowy
                    stopp = 0;
                }
                aa = 1;
            }
            if (aa == 2)
            {
                aa = 0;
                stopp++;
                pcf8574(cyfrybledow[0]); /// ledy wyłączone
            }
            czasbledu = czas;
        }
        if (bladA == 0 && bladB == 0 && bladC == 0 && bladD == 0 && bladE == 0 && bladF == 0 && bladG == 0 && bladH == 0 && bladJ == 0 && bladI == 0 &&
            bladK == 0 && bladL == 0 && bladM == 0 && bladN == 0 && bladR == 0 && bladRR == 0 && bladS == 0 && bladSS == 0 &&
            bladRR_R == 0 && bladSS_S == 0)
        {
            pcf8574(cyfrybledow[0]); /// ledy wyłączone
            blad = 0;
            stopp = 1;
            aa = 0;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// BUZZER
    if (buzzerwlacz == 1)
    {
        if (blad == 0)
        {
            if (werpikacz3 == 1)
            {
                digitalWrite(buzzer, 0);
                werpikacz3 = 0;
            }

            if (czas - czaspikaczu1 < (unsigned long)200)
            { //  /pikanie przyciski i pilot  krutkie
                if (werpikacz1 == 0)
                {
                    digitalWrite(buzzer, 1);
                    werpikacz1 = 1;
                }
            }
            else
            {
                if (werpikacz1 == 1 && werpikacz2 == 0)
                {
                    digitalWrite(buzzer, 0);
                    werpikacz1 = 0;
                }
            }

            if (czas - czaspikaczu2 < (unsigned long)800)
            { //  zatwierdzenie opuźnie zamykanie  długie
                if (werpikacz2 == 0)
                {
                    digitalWrite(buzzer, 1);
                    werpikacz2 = 1;
                }
            }
            else
            {
                if (werpikacz2 == 1)
                {
                    digitalWrite(buzzer, 0);
                    werpikacz2 = 0;
                }
            }
        }
        else
        {
            if (czas - czaspikaczu3 > (unsigned long)80)
            { //  pikanie
                werpikacz3 = !werpikacz3;
                digitalWrite(buzzer, werpikacz3);
                czaspikaczu3 = czas;
            }
        }
    }
}
///////KONIEC///////////////KONIEC///////////////KONIEC///////////////KONIEC///////////////KONIEC///////////////KONIEC///////////////KONIEC///////////////KONIEC///////////////KONIEC///////////////KONIEC////////