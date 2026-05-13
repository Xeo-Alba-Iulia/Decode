-keepattributes RuntimeVisibleAnnotations
-keep @interface com.acmerobotics.dashboard.config.Config
-keep @interface com.qualcomm.robotcore.eventloop.opmode.*
-keep @interface org.firstinspires.ftc.ftccommon.external.*

-keep,allowoptimization @com.qualcomm.robotcore.eventloop.opmode.Autonomous class **
-keep,allowoptimization @com.qualcomm.robotcore.eventloop.opmode.TeleOp class **

-keepclasseswithmembers,allowobfuscation,allowoptimization class ** {
    @org.firstinspires.ftc.ftccommon.external.* <methods>;
}
-keepclasseswithmembers,allowobfuscation,allowoptimization class ** {
    @com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar <methods>;
}

-keepnames @com.acmerobotics.dashboard.config.Config class ** {
    public static !final <fields>;
}

-optimizationpasses 10
-overloadaggressively
-repackageclasses 'o'
-allowaccessmodification

-whyareyoukeeping class org.firstinspires.ftc.vision.VisionPortal

-dontwarn javax.xml.stream.XMLInputFactory
-dontwarn javax.xml.stream.XMLStreamReader
