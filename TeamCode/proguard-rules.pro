-keep @com.qualcomm.robotcore.eventloop.opmode.Autonomous class **
-keep @com.qualcomm.robotcore.eventloop.opmode.TeleOp class **

-keepclassmembers class ** {
    @org.firstinspires.ftc.robotcore.external.OnCreate <methods>;
}
-keepclassmembers class ** {
    @org.firstinspires.ftc.robotcore.external.OnCreateEventLoop <methods>;
}
-keepclassmembers class ** {
    @org.firstinspires.ftc.robotcore.external.OnCreateMenu <methods>;
}
-keepclassmembers class ** {
    @org.firstinspires.ftc.robotcore.external.OnDestroy <methods>;
}
-keepclassmembers class ** {
    @org.firstinspires.ftc.robotcore.external.WebHandlerRegistrar <methods>;
}

-dontwarn javax.xml.stream.XMLInputFactory
-dontwarn javax.xml.stream.XMLStreamReader
