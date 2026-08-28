const translations = {
    "About": "О проекте",
    "3D Printing": "3D-печать",
    "Components": "Компоненты",
    "Assembly": "Сборка",
    "Firmware": "Прошивка",
    "GUI": "Графический интерфейс",
    "The story of the hovercraft, which we named Cheburashka, began in September 2025. As part of the Introduction to Digital Manufacturing course, we were tasked with building a device with a non-standard design. That’s how I came up with the idea to build Cheburashka.": "История воздушной подушки, которую мы назвали <span class=\"highlight\">Cheburashka</span>, началась в сентябре 2025 года. В рамках курса <span class=\"highlight\">«Введение в цифровое производство»</span> нам поручили создать устройство с нестандартной ходовой. Так появилась идея Cheburashka.",
    "I didn’t want to copy anything from the internet, so I built it exactly as I had imagined it. By that point, I had already been working with drones and knew how to design models in Blender, so the development process got off to a quick start. Yes, Blender is the most user-friendly program for me, and it’s fairly easy to turn it into a decent CAD system using add-ons. But most importantly, Blender is very forgiving of mistakes, and that will save you more than once.": "<span class=\"highlight\">Мы не хотели копировать что-либо из интернета, поэтому создали проект именно таким, каким я его представлял.</span> К тому моменту я уже работал с дронами и умел проектировать модели в <span class=\"highlight\">Blender</span>, поэтому разработка началась быстро. <span class=\"highlight\">Blender — самая удобная для меня программа, а с помощью дополнений её легко превратить в достойную CAD-систему.</span> Blender очень удобен для новичков.",
    "All the latest news is posted on our Telegram channel": "Все последние новости публикуются в нашем <a href=\"https://t.me/pcp_podushka\">Telegram-канале</a>",
    "All 3D models were created in Blender. In total, three versions of Cheburashka were created, and perhaps this is not the end yet. Each time, some shortcomings would appear that had to be fixed in subsequent version": "Все 3D-модели созданы в Blender. Всего было разработано три версии Cheburashka, и, возможно, это ещё не конец. В каждой версии обнаруживались недостатки, которые приходилось исправлять в следующей.",
    "Here is the latest, highest‑quality, and most well‑thought‑out version. The Cheburashka body is waterproof if the engineering hole for the wires of the main cushion motor is sealed.": "Перед вами последняя, наиболее качественная и продуманная версия. Корпус Cheburashka водонепроницаем, если загерметизировать технологическое отверстие для проводов двигателя основной подушки.",
    "I printed all the parts on an Elegoo Neptune Plus 3D printer using PLA and PETG plastic.": "Все детали напечатаны на 3D-принтере Elegoo Neptune Plus из PLA и PETG.",
    "All that I list below is everything that is in my Cheburashka, this is not a recommendation for purchase and you can find better modules. After the list, I'll tell you what can be improved by modules.": "Ниже перечислены все компоненты моей Cheburashka. Это не рекомендация к покупке — можно найти модули лучше. После списка я расскажу, что можно улучшить.",
    "So, now a few of my recommendations. Of course, the ESP32 must be equipped with a Wi-Fi module. I also advise you to purchase a pin expander for the board.": "Несколько рекомендаций. ESP32 должна быть оснащена Wi-Fi-модулем. Также советую приобрести расширитель выводов для платы.",
    "There are much more interesting radio transmitters, but I still advise you to look at the information about the ExpressLRS firmware. This firmware is also suitable for FrSky R9 Mini OTA. Such a connection can operate at a distance of up to ~15 km, which ensures high-quality signal reception and safe operation.": "Есть более интересные радиопередатчики, но я советую изучить прошивку ExpressLRS. Она подходит и для FrSky R9 Mini OTA. Такая связь работает на расстоянии до 15 км, обеспечивая качественный приём сигнала и безопасную эксплуатацию.",
    "The battery can be from 4S to 6S. When using the 4S, you will be able to install a larger battery. I also advise you to buy shrink wrap and nylon wires of different thicknesses.": "Аккумулятор может быть от 4S до 6S. При использовании 4S можно установить аккумулятор большей ёмкости. Также советую приобрести термоусадку и нейлоновые провода разного сечения.",
    "I recommend buying rotary motors with a characteristic of the order of 2000-3000 KV. It will be more comfortable for 3.5\" diameter propellers. The main engine should be of the order of 1500-2000 KV.": "Рекомендую приобрести поворотные двигатели с характеристикой около 2000–3000 KV. С ними будет удобнее использовать пропеллеры диаметром 3,5 дюйма. Основной двигатель должен иметь характеристику около 1500–2000 KV.",
    "M3 bolts are used. Buy nylon ones about 1.5 cm long — you can always cut them to the desired length — as well as metal ones about 1 cm long. Don't forget about nuts and 4-mm-high solder-in bushings": "Используются болты M3. Купите нейлоновые длиной около 1,5 см — их всегда можно укоротить, — а также металлические длиной около 1 см. Не забудьте гайки и впаиваемые втулки высотой 4 мм.",
    "So, to assemble the model, you’ll need 16 M3 nylon bolts, each 1.5 cm long. Why nylon? Because you can cut them to the desired length, which is very convenient. You’ll also need about 20 4-mm-tall solder-in bushings for M3 bolts.": "Для сборки модели понадобятся 16 нейлоновых болтов M3 длиной 1,5 см. Нейлон удобен тем, что болты можно укоротить. Также понадобится около 20 впаиваемых втулок высотой 4 мм для болтов M3.",
    "First, you need to glue the bottom in place so that the body is airtight. For this, I used PU-40 polyurethane sealant.": "Сначала приклейте днище, чтобы корпус был герметичным. Для этого я использовал полиуретановый герметик PU-40.",
    "Next, carefully solder the solder-in bushings in place using a soldering iron. I didn’t have a special tool for this, but you can use a long bolt, screw the solder-in bushing onto it, heat it with a soldering iron, and hold the bolt in place with pliers. I think it’s obvious where to solder them in, but I’ve marked the spots with red circles just in case.": "Затем аккуратно впаяйте втулки паяльником. У меня не было специального инструмента, но можно использовать длинный болт: накрутить на него впаиваемую втулку, нагреть её паяльником и удерживать болт плоскогубцами. Думаю, понятно, куда их впаивать, но на всякий случай я отметил места красными кругами.",
    "Do the same carefully on top and in the braces.": "Так же аккуратно установите втулки сверху и в распорках.",
    "Attach the lugs. I think it’s pretty clear how to position them to fit into the holes. Tighten the bolts onto the nuts. It’s quite possible that the M3 bolts won’t fit through the holes, and that’s perfectly normal—just drill them out if necessary.": "Установите уши и их держатели, совместив их с отверстиями. Затяните болты на гайках. Если болты M3 не проходят в отверстия, просто рассверлите отверстия.",
    "Next, you need to screw on the motors. They usually come with bolts; I recommend screwing a nylon nut onto these bolts. During prolonged operation, they tend to heat up from the motor windings, and without you noticing, they can melt through the mounting point, so the nylon nut acts as a buffer.": "Затем установите двигатели. Обычно они поставляются с болтами; рекомендую накрутить на эти болты нейлоновые гайки. При длительной работе болты нагреваются от обмоток двигателя и могут незаметно расплавить место крепления, поэтому нейлоновая гайка служит прокладкой.",
    "Fit the main motor housing in place, aligning the holes so that the wires can be passed through them. Next, screw in the main motor, seal its housing, and seal the opening in the body where the wires pass through.": "Установите корпус основного двигателя, совместив отверстия так, чтобы через них можно было провести провода. Затем закрепите основной двигатель, загерметизируйте его корпус и отверстие в корпусе, через которое проходят провода.",
    "So, you’ve assembled the Cheburashka model. Now let’s talk about the electronics.": "Итак, модель Cheburashka собрана. Теперь поговорим об электронике.",
    "I strongly recommend purchasing heat-shrink tubing in various diameters in advance. For the power wires, you’ll need XT60 connectors and wires with a fairly large gauge—I’m using wires with a diameter of about 3 mm. For connecting the modules, wires with a smaller gauge, about 1 mm, will suffice.": "Заранее рекомендую приобрести термоусадку разных диаметров. Для силовых проводов понадобятся разъёмы XT60 и толстые провода — я использую провода диаметром около 3 мм. Для подключения модулей достаточно проводов диаметром около 1 мм.",
    "I soldered everything using a TS100 soldering iron. Also, buy solder flux and solder. For solder, I recommend MECHANIC TY-V866.": "Всё спаивалось паяльником TS100. Также понадобятся флюс и припой. Рекомендую припой MECHANIC TY-V866.",
    "Below is a table and a wiring diagram. It’s hard to get confused here. Just look at the labels on the module and the ESP32. There’s no need to say anything more.": "Ниже приведены таблица и схема подключения. Здесь сложно запутаться: просто смотрите на обозначения на модуле и ESP32. Больше здесь нечего добавить.",
    "Model unfolding:": "Развёртка модели:",
    "Part 1: Assemble Model": "Часть 1: сборка модели",
    "Part 2: Assemble Electronics": "Часть 2: сборка электроники",
    "Install the necessary libraries:": "Установите необходимые библиотеки:",
    "Module pin": "Контакт модуля",
    "ESP32 pin": "Контакт ESP32",
    "Now let's talk about how to flash esp32. I recommend doing this on a Unix system.": "Теперь поговорим о прошивке ESP32. Рекомендуется выполнять это в Unix-системе.",
    "Install the Arduino IDE.": "Установите <a href=\"https://www.arduino.cc/en/software\">Arduino IDE</a>.",
    "Compile it. If the compilation is successful, connect the esp32 and download the firmware on ESP32.": "Скомпилируйте проект. Если компиляция прошла успешно, подключите ESP32 и загрузите прошивку.",
    "Please read the article. You need to download the firmware_vars.json on ESP32.": "Прочитайте <a href=\"https://randomnerdtutorials.com/esp32-littlefs-arduino-ide/\">статью</a>. Загрузите <a href=\"../firmwares/esp32_firmware/data/firmware_vars.json\">firmware_vars.json</a> на ESP32.",
    "Open the esp32_firmware.ino file in IDE.": "Откройте файл <a href=\"../firmwares/esp32_firmware/esp32_firmware.ino\">esp32_firmware.ino</a> в IDE.",
    "Download the latest Cheburashka GUI from the v1.0.0 release.": "Скачайте последнюю версию GUI Cheburashka из <a href=\"https://github.com/Matvey787/Airbag/releases/tag/v1.0.0\">релиза v1.0.0</a>.",
    "All models you can find here.": "Все модели доступны здесь.",
    "Success! You’ve assembled Cheburashka model.": "Готово! Вы собрали модель Cheburashka.",
    "Next, attach the skeleton from below.": "Прикрепите каркас снизу.",
    "The two remaining parts are needed to clamp the pocket between them.": "Две оставшиеся детали нужны для зажима юбки между ними."
};

const languageSwitch = document.querySelector(".language-switch");
const elements = [...document.querySelectorAll("h1, h2, p, th, nav a, li:not(:has(a)), li[data-i18n]")];
const originalText = elements.map((element) => ({
    element,
    key: element.dataset.i18n || element.textContent.trim(),
    html: element.innerHTML
}));

function setLanguage(language) {
    document.documentElement.lang = language;
    originalText.forEach(({element, key, html}) => {
        if (language === "ru" && translations[key]) {
            element.innerHTML = translations[key];
        } else if (language === "en") {
            element.innerHTML = html;
        }
    });
    languageSwitch.textContent = language === "en" ? "RU" : "EN";
    languageSwitch.setAttribute("aria-label", language === "en"
        ? "Switch to Russian"
        : "Switch to English");
    localStorage.setItem("cheburashka-language", language);
}

languageSwitch.addEventListener("click", () => {
    setLanguage(document.documentElement.lang === "en" ? "ru" : "en");
});
setLanguage(localStorage.getItem("cheburashka-language") || "en");
