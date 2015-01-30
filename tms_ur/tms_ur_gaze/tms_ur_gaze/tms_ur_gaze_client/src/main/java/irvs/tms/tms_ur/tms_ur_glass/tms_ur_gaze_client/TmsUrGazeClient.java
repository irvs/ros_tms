package irvs.tms.tms_ur.tms_ur_glass.tms_ur_gaze_client;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.util.ArrayList;

/**
 * tms_ur_glass_vicon client
 *
 *  - Speech recognition
 *     - Executes recognition services on worker threads, using SpeechRecognizer intent
 *     - Publishes to the topic, /recognition_text; another c++ nodes subscribes it.
 *
 *  - Updating CameraView
 *     - For each frame, captured images are set as background image after processing
 */

public class TmsUrGazeClient extends RosActivity implements View.OnClickListener,RecognitionListener
{
    private java.lang.Object mSpeechRecognizer;

    private RecognitionClient recognitionClient;

    private Intent intent;
    private TextView guiMessage;
    public static TextView guiResult;

    Handler guiThreadHandler;

    /**
     * Default constructor
     */
    public TmsUrGazeClient() {
        super("Speech Recognition Test","Speech Recognition Test");
    }

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        Button button1 = (Button)findViewById(R.id.button);
        button1.setOnClickListener(this);
        guiMessage = (TextView)findViewById(R.id.message);
        guiResult = (TextView) findViewById(R.id.result);

        guiThreadHandler = new Handler();

        mSpeechRecognizer = SpeechRecognizer.createSpeechRecognizer(this);
        ((SpeechRecognizer) mSpeechRecognizer).setRecognitionListener(this);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration
                = NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        recognitionClient = new RecognitionClient();
        nodeMainExecutor.execute(recognitionClient, nodeConfiguration);
    }

    @Override
    public void onClick(View v) {
        /**
         * Executes speech recognition service
         * if the button selected on UI thread.
         */
        switch (v.getId()) {
            case R.id.button:
                intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
                intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
                intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, getPackageName());
                ((SpeechRecognizer) mSpeechRecognizer).startListening(intent);
                break;
            default:
                break;
        }
    }

    //---------------------------------------------------------------------------------------------------------------
    /**
     * Overridden functions for SpeechRecognizer intent
     *
     */
    @Override
    public void onReadyForSpeech(Bundle params) {
        guiMessage.setText("onReadyForSpeech");
    }

    @Override
    public void onBeginningOfSpeech() {
        guiMessage.setText("onBeginningOfSpeech");
    }

    @Override
    public void onBufferReceived(byte[] buffer) {
        Log.v("DEBUG", "onBufferReceived");
    }

    @Override
    public void onRmsChanged(float rmsdB) {
        Log.v("DEBUG","receive : " + rmsdB + "dB");
    }

    @Override
    public void onEndOfSpeech() {
        guiMessage.setText("onEndOfSpeech");
    }

    @Override
    public void onError(int error) {
        switch (error) {
            case SpeechRecognizer.ERROR_AUDIO:
                guiMessage.setText("ERROR_AUDIO");
                break;
            case SpeechRecognizer.ERROR_CLIENT:
                guiMessage.setText("ERROR_CLIENT");
                break;
            case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                guiMessage.setText("ERROR_INSUFFICIENT_PERMISSIONS");
                break;
            case SpeechRecognizer.ERROR_NETWORK:
                guiMessage.setText("ERROR_NETWORK");
                break;
            case SpeechRecognizer.ERROR_NETWORK_TIMEOUT:
                guiMessage.setText("ERROR_NETWORK_TIMEOUT");
                break;
            case SpeechRecognizer.ERROR_NO_MATCH:
                guiMessage.setText("ERROR_NO_MATCH");
                break;
            case SpeechRecognizer.ERROR_RECOGNIZER_BUSY:
                guiMessage.setText("ERROR_RECOGNIZER_BUSY");
                break;
            case SpeechRecognizer.ERROR_SERVER:
                guiMessage.setText("ERROR_SERVER");
                break;
            case SpeechRecognizer.ERROR_SPEECH_TIMEOUT:
                guiMessage.setText("ERROR_SPEECH_TIMEOUT(No input)");
                break;
            default:
        }
    }

    @Override
    public void onEvent(int eventType, Bundle params) {
        Log.v("DEBUG", "onEvent");
    }

    @Override
    public void onPartialResults(Bundle partialResults) {
        /**
         * For Vuzix M100 smart glass.
         * M100 has Nuance's voice recognition engine.
         * never calls onResults()
         */
        guiMessage.setText("onPartialResults");
        Log.v("DEBUG", "onPartialResults");

        ArrayList<String> recData = partialResults.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
        final String[] str_items = recData.toArray(new String[recData.size()]);
        recognitionClient.sendText(str_items[0]);
    }

    @Override
    public void onResults(Bundle results) {
        ArrayList<String> recData = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
        final String[] str_items = recData.toArray(new String[recData.size()]);

        Log.i("ROS:TmsUrGazeClient","Send recognized result to server");
        recognitionClient.sendText(str_items[0]);
    }
}