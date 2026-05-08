-keep @com.qualcomm.robotcore.eventloop.opmode.Autonomous class **
-keep @com.qualcomm.robotcore.eventloop.opmode.TeleOp class **

-keepclassmembers class ** {
    @org.firstinspires.ftc.ftccommon.external.OnCreate <methods>;
}
-keepclassmembers class ** {
    @org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop <methods>;
}
-keepclassmembers class ** {
    @org.firstinspires.ftc.ftccommon.external.OnCreateMenu <methods>;
}
-keepclassmembers class ** {
    @org.firstinspires.ftc.ftccommon.external.OnDestroy <methods>;
}
-keepclassmembers class ** {
    @org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar <methods>;
}

-dontwarn javax.xml.stream.XMLInputFactory
-dontwarn javax.xml.stream.XMLStreamReader
