/*
 * main_sadece_kurtarma_algoritmasi.c
 *
 *  Created on: 21 Şub 2026
 *      Author: ayberk
 *      sadece kurtarma algoritması olan kodu, main.c kodu içine entegre etmeden önceki halini de ekledim.
 */


#include "bno055.h"
#include <string.h>
#include <stdio.h>

//fsm(final state machine) mantığı bu tarz sistemler için biçilmiş kaftanır. bu yüzden burada durum tanımlamalarını yaptık
typedef enum {
    ON_RAMP,      // rampada
    IN_FLIGHT,     // roket uçuşda, irtifa artarken
    DRAG_PARACHUTE_OPEN,    // apogee tespitinden sonra sürüklenme paraşütünü açmak için
    MAIN_PARACHUTE_OPEN,       // ana paraşütü açmak ve iniş yapmak için
    LANDED    // yere iniş tamamlanınca
} state;

// global değişkenler
state currentState = ON_RAMP; //başta rampadan başlatıyoruz
float currentAltitude = 0.0f; //işlemciyi yormamak adına sonunda f koyduk ki double ile işlem yapmasın, hep float kullanılsın.
float maxAltitude = 0.0f;
float previousAltitude = 0.0f;

// BNO055den veri çekmek için kullanacağımız yapıyı oluşturuyoruz, sonrasında tek tek x,y ve z eksenlerinden veri çekmek için orientation.x şeklinde structtan veri çekeceğiz
bno055_vector_t orientation;

//paraşütleri açacak komutu göndermek için kullanacağımız fonksiyonlar elimde olmadığı için buralara dummy kullanacağım
void drag_parachute_open() {

}

void main_parachute_open() {

}

// ana kontrol döngümüz
void ucus_algoritmasi() {

    // ms5611'den irtifa verisi alıyoruz
    currentAltitude = ms5611_get_altitude();

    // bno055'ten euler açılarını okuyoruz ve oluşturduğumuz orientation yapısına yazdırıyoruz.
    orientation = bno055_getVector(BNO055_VECTOR_EULER);

    // Final State Machine'miz
    switch(currentState) {

        case ON_RAMP:
            // irtifa kontrolü, 10 m'den yüksekdeysek uçuyoruz demektir.
            if (currentAltitude > 10.0f) {
                currentState = IN_FLIGHT;
            }
            break;

        case IN_FLIGHT:
            if (currentAltitude > maxAltitude) {
                maxAltitude = currentAltitude;
            }

            // EK KONTROL, eğer roketin dikliği (euler açıslarına bakarak karar veriyoruz) 80 dereceden fazla saparsa acil durum kurtarmasını başlatacağız. (z ekseni genelde dikeyi temsil eder bno055 kütüphanelerinde ama sensörün nasıl monte edildiğine bağlı olarak da değişebilir, bu donanımla beraber ayarlanacak bir iştir.)
            if (orientation.z > 80.0 || orientation.z < -80.0) {
                drag_parachute_open();
                currentState = DRAG_PARACHUTE_OPEN;
            }

            // apogee tespiti yaptık; eğer irtifa düşmeye başladıysa apogee'ye ulaşmışım ve hatta düşmeye başlamışım demektir
            if (currentAltitude < (maxAltitude - 2.0f)) {
                drag_parachute_open();
                currentState = DRAG_PARACHUTE_OPEN;
            }
            break;

        case DRAG_PARACHUTE_OPEN:
            // ana paraşüt açılma irtifası (örnekte 500 metre olsun dedim)
            if (currentAltitude < 500.0f) {
                main_parachute_open();
                currentState = MAIN_PARACHUTE_OPEN;
            }
            break;

        case MAIN_PARACHUTE_OPEN:
            // iniş tespiti yaptık; irtifa yere yakınsa LANDED state'ine geçtik.
            if (currentAltitude < 10.0f) {
                currentState = LANDED;
            }
            break;

        case LANDED:
            // telemetri ekranına indi verisi vs gönderilebilir
            break;
    }

    // yükseklik değerini güncelliyoruz
    previousAltitude = currentAltitude;
}
