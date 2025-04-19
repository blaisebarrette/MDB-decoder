using System;
using System.Collections.Generic;
using System.ComponentModel.Composition;
using System.Linq;
using LabNation.Interfaces;
using System.Text; // Added for StringBuilder

namespace LabNation.Decoders
{
    [Export(typeof(IDecoder))]
    public class DecoderMDB : IDecoder
    {
        public DecoderDescription Description
        {
            get
            {
                return new DecoderDescription()
                {
                    Name = "MDB Decoder",
                    ShortName = "MDB",
                    Author = "Blaise", // Votre nom ici !
                    VersionMajor = 0,
                    VersionMinor = 2, // Incremented version
                    Description = "Decodes the Multi-Drop Bus (MDB/ICP) protocol, commonly used in vending machines.",
                    InputWaveformTypes = new Dictionary<string, Type>()
                    {
                        { "MDB Line", typeof(bool)} // Unique ligne de données MDB
                    },
                    Parameters = null // Pas de paramètres pour l'instant
                };
            }
        }

        // Constantes MDB
        private const double BAUD_RATE = 9600.0;
        private const double BIT_DURATION_SECONDS = 1.0 / BAUD_RATE;
        private const int SAMPLES_PER_BIT = 10; // Minimum samples to check within a bit time for stability
        private const double START_BIT_CHECK_DURATION_FACTOR = 0.4; // Check start bit stability for 40% of bit duration
        private const double SAMPLE_POINT_FACTOR = 0.5; // Sample data bits at 50% of their duration
        private const double MAX_INTERBYTE_TIME_SECONDS = 0.001; // 1 ms - Max time between bytes within a block

        // Special MDB Bytes
        private const byte MDB_ACK = 0x00;
        private const byte MDB_NAK = 0xFF;

        // Adresses MDB connues (à compléter si nécessaire)
        private enum MdbAddress : byte
        {
            VMC = 0x00,             // Master
            Changer = 0x08,
            Cashless1 = 0x10,
            Gateway = 0x18,
            BillValidator = 0x30,
            Cashless2 = 0x60,
            Experimental1 = 0xE0,
            Experimental2 = 0xE8,
            Specific1 = 0xF0,
            Specific2 = 0xF8,
        }

        // Structure pour stocker le résultat de la lecture d'une trame
        private struct MdbFrame
        {
            public byte Data;
            public bool ModeBit;
            public int StartIndex;
            public int EndIndex;
            public bool FramingError;
            public bool StartBitGlitch; // Flag if start bit was noisy
        }

        // Renamed from Decode to Process to match IDecoder interface
        DecoderOutput[] IDecoder.Process(Dictionary<string, Array> inputWaveforms, Dictionary<string, object> parameters, double samplePeriod) 
        {
            if (!inputWaveforms.ContainsKey("MDB Line") || !(inputWaveforms["MDB Line"] is bool[]))
                throw new ArgumentException("Input 'MDB Line' of type bool[] is required.");

            bool[] mdbLine = (bool[])inputWaveforms["MDB Line"];
            List<DecoderOutput> outputList = new List<DecoderOutput>();
            int currentIndex = 0;
            int samplesPerBitPeriod = (int)Math.Max(1, Math.Round(BIT_DURATION_SECONDS / samplePeriod)); // Samples in one bit time
            double maxInterByteSamples = MAX_INTERBYTE_TIME_SECONDS / samplePeriod;

            List<MdbFrame> decodedFrames = new List<MdbFrame>(); // Store frames before processing blocks

            // --- Step 1: Decode individual frames --- 
            while (currentIndex < mdbLine.Length - samplesPerBitPeriod * 11) // Ensure enough data for a full frame
            {
                int startBitIndex = FindNextStableFallingEdge(mdbLine, currentIndex, samplesPerBitPeriod);
                if (startBitIndex < 0) break; // No more stable start bits found

                MdbFrame frame = TryReadMdbFrame(mdbLine, startBitIndex, samplePeriod, samplesPerBitPeriod);

                if (frame.FramingError)
                {
                    outputList.Add(new DecoderOutputEvent(startBitIndex, frame.EndIndex > startBitIndex ? frame.EndIndex : startBitIndex + samplesPerBitPeriod, DecoderOutputColor.Red, "Framing Err"));
                    currentIndex = startBitIndex + 1; 
                }
                else
                {
                    if (frame.StartBitGlitch)
                    {
                        outputList.Add(new DecoderOutputEvent(startBitIndex, frame.EndIndex, DecoderOutputColor.Orange, "Glitch?"));
                    }
                    decodedFrames.Add(frame);
                    currentIndex = frame.EndIndex; 
                }
            }

            // --- Step 2: Process decoded frames into blocks --- 
            ProcessDecodedFrames(decodedFrames, outputList, samplePeriod, maxInterByteSamples);

            return outputList.ToArray();
        }

        // Cherche une transition H->L stable (reste L pendant une partie du bit)
        private int FindNextStableFallingEdge(bool[] data, int startIndex, int samplesPerBit)
        {
            int checkSamples = (int)Math.Max(1, samplesPerBit * START_BIT_CHECK_DURATION_FACTOR);

            for (int i = startIndex; i < data.Length - samplesPerBit; i++) // Need at least one bit duration after edge
            {
                if (data[i] && !data[i + 1])
                {
                    int potentialStart = i + 1;
                    bool stable = true;
                    for (int k = 1; k < checkSamples && potentialStart + k < data.Length; k++)
                    {
                        if (data[potentialStart + k])
                        {
                            stable = false;
                            i = potentialStart + k -1;
                            break;
                        }
                    }
                    if (stable) return potentialStart;
                }
            }
            return -1; // Not found
        }

        // Tente de lire une trame MDB de 11 bits
        private MdbFrame TryReadMdbFrame(bool[] data, int startBitIndex, double samplePeriod, int samplesPerBit)
        {
            MdbFrame result = new MdbFrame { StartIndex = startBitIndex, EndIndex = startBitIndex, FramingError = true, StartBitGlitch = false };
            int samplePointOffset = (int)Math.Max(1, samplesPerBit * SAMPLE_POINT_FACTOR); // Sample point within the bit

            for(int i = 1; i < samplesPerBit; i++)
            {
                if (startBitIndex + i >= data.Length || data[startBitIndex + i])
                {
                    result.StartBitGlitch = true; 
                }
            }

            byte dataByte = 0;
            bool modeBit = false;
            bool stopBit = false;
            result.EndIndex = startBitIndex + 11 * samplesPerBit;

            if (result.EndIndex >= data.Length)
            {
                 result.EndIndex = data.Length -1;
                 return result; // Not enough data
            }

            try
            {
                for (int bit = 0; bit < 8; bit++)
                {
                    int sampleIndex = startBitIndex + (bit + 1) * samplesPerBit + samplePointOffset;
                    if (data[sampleIndex]) dataByte |= (byte)(1 << bit);
                }
                int modeBitSampleIndex = startBitIndex + 9 * samplesPerBit + samplePointOffset;
                modeBit = data[modeBitSampleIndex];

                int stopBitSampleIndex = startBitIndex + 10 * samplesPerBit + samplePointOffset;
                stopBit = data[stopBitSampleIndex]; 
            }
            catch (IndexOutOfRangeException) { return result; }

            if (!stopBit) return result; // Framing error - Stop bit is Low
            
            result.Data = dataByte;
            result.ModeBit = modeBit;
            result.FramingError = false; // Success!
            return result;
        }

        // Calcule le checksum MDB pour une liste de trames
        private byte CalculateChecksum(List<MdbFrame> blockFrames)
        {
            byte checksum = 0;
            foreach (var frame in blockFrames)
            {
                checksum += frame.Data;
            }
            return checksum;
        }

        // Assemble les trames en blocs et génère les DecoderOutputs
        private void ProcessDecodedFrames(List<MdbFrame> frames, List<DecoderOutput> outputList, double samplePeriod, double maxInterByteSamples)
        {
            if (frames.Count == 0) return;

            List<MdbFrame> currentBlock = new List<MdbFrame>();
            int lastFrameEndIndex = 0;

            for (int i = 0; i < frames.Count; i++)
            {
                MdbFrame currentFrame = frames[i];

                // Vérifier le timing inter-octets (si on est déjà dans un bloc)
                if (currentBlock.Count > 0)
                {
                    double samplesBetweenFrames = currentFrame.StartIndex - lastFrameEndIndex;
                    if (samplesBetweenFrames > maxInterByteSamples)
                    {
                        // Timeout inter-octets: le bloc précédent est invalide/interrompu
                        if (currentBlock.Count > 0)
                        {
                             outputList.Add(new DecoderOutputEvent(currentBlock.First().StartIndex, lastFrameEndIndex, DecoderOutputColor.Orange, "Block Timeout?"));
                             currentBlock.Clear();
                        }
                    }
                }

                // Logique d'assemblage des blocs
                if (currentBlock.Count == 0) // Début d'un potentiel nouveau bloc/ACK/NAK
                {
                    if (currentFrame.ModeBit) // Mode=1: Peut être Adresse (Master) ou ACK/NAK/CHK (Slave)
                    {
                        if (currentFrame.Data == MDB_ACK) // C'est un ACK seul
                        {
                             outputList.Add(new DecoderOutputEvent(currentFrame.StartIndex, currentFrame.EndIndex, DecoderOutputColor.Green, "ACK"));
                             // Ne pas démarrer de bloc
                        }
                        else if (currentFrame.Data == MDB_NAK) // C'est un NAK seul
                        {
                             outputList.Add(new DecoderOutputEvent(currentFrame.StartIndex, currentFrame.EndIndex, DecoderOutputColor.Red, "NAK"));
                             // Ne pas démarrer de bloc
                        }
                        else // C'est un octet d'Adresse (début de bloc Maître)
                        {
                            currentBlock.Add(currentFrame);
                        }
                    }
                    else // Mode=0: Ne peut être que le début d'un bloc de données Esclave (rare, mais possible si on rate le début)
                    {
                        currentBlock.Add(currentFrame); // Suppose début de bloc Esclave
                    }
                }
                else // Dans un bloc existant
                {
                    byte expectedChecksum = 0;
                    bool checksumOk = false;
                    bool blockComplete = false;
                    bool isMasterBlock = currentBlock[0].ModeBit; // Le premier octet détermine si c'est un bloc Maître

                    if (isMasterBlock)
                    {
                        // Bloc Maître: Se termine par un CHK (Mode=0)
                        if (!currentFrame.ModeBit) // Mode=0 -> C'est le CHK
                        {
                            currentBlock.Add(currentFrame); // Ajoute le CHK au bloc
                            expectedChecksum = CalculateChecksum(currentBlock.GetRange(0, currentBlock.Count - 1)); // Checksum sur Adresse + Données
                            checksumOk = (currentFrame.Data == expectedChecksum);
                            blockComplete = true;

                            string dataStr = string.Join(" ", currentBlock.GetRange(1, currentBlock.Count - 2).Select(f => $"0x{f.Data:X2}"));
                            string title = $"Master Block [Addr: 0x{currentBlock[0].Data:X2}] Data: {dataStr} CHK: 0x{currentFrame.Data:X2}";
                            outputList.Add(new DecoderOutputEvent(currentBlock.First().StartIndex, currentFrame.EndIndex,
                                                                checksumOk ? DecoderOutputColor.Blue : DecoderOutputColor.Red,
                                                                checksumOk ? title : title + " (Error! Expected: 0x" + expectedChecksum.ToString("X2") + ")"));
                        }
                        else // Mode=1 -> Erreur de protocole, on attendait Mode=0 (Data ou CHK)
                        {
                             outputList.Add(new DecoderOutputEvent(currentBlock.First().StartIndex, currentFrame.EndIndex, DecoderOutputColor.Red, "Protocol Error (Master Block: Expected Mode=0)"));
                             blockComplete = true; // Abandonne le bloc actuel
                        }
                    }
                    else // Bloc Esclave en cours (commencé par Mode=0)
                    {
                        // Bloc Esclave: Se termine par un octet avec Mode=1 (ACK, NAK, ou CHK)
                        if (currentFrame.ModeBit) // Mode=1 -> Fin de bloc Esclave
                        {
                             blockComplete = true;
                             if (currentFrame.Data == MDB_ACK) // Fin par ACK
                             {
                                 // Ne devrait pas arriver si le bloc a des données, ACK est seul
                                 outputList.Add(new DecoderOutputEvent(currentBlock.First().StartIndex, currentFrame.EndIndex, DecoderOutputColor.Orange, "Slave Data followed by ACK?"));
                             }
                             else if (currentFrame.Data == MDB_NAK) // Fin par NAK
                             {
                                 // Ne devrait pas arriver si le bloc a des données, NAK est seul
                                  outputList.Add(new DecoderOutputEvent(currentBlock.First().StartIndex, currentFrame.EndIndex, DecoderOutputColor.Orange, "Slave Data followed by NAK?"));
                             }
                             else // Fin par CHK (Mode=1)
                             {
                                currentBlock.Add(currentFrame); // Ajoute le CHK
                                expectedChecksum = CalculateChecksum(currentBlock.GetRange(0, currentBlock.Count - 1)); // Checksum sur toutes les données précédentes
                                checksumOk = (currentFrame.Data == expectedChecksum);

                                string dataStr = string.Join(" ", currentBlock.GetRange(0, currentBlock.Count - 1).Select(f => $"0x{f.Data:X2}"));
                                string title = $"Slave Block Data: {dataStr} CHK: 0x{currentFrame.Data:X2}";
                                outputList.Add(new DecoderOutputEvent(currentBlock.First().StartIndex, currentFrame.EndIndex,
                                                                    checksumOk ? DecoderOutputColor.Green : DecoderOutputColor.Red,
                                                                    checksumOk ? title : title + " (Error! Expected: 0x" + expectedChecksum.ToString("X2") + ")"));
                             }
                        }
                        else // Mode=0 -> Octet de données Esclave supplémentaire
                        {
                             currentBlock.Add(currentFrame);
                             // Reste dans l'état ReceivingSlaveBlock
                        }
                    }

                    if (blockComplete)
                    {
                         currentBlock.Clear(); // Prêt pour le prochain bloc
                    }
                }

                lastFrameEndIndex = currentFrame.EndIndex;
            } // Fin de la boucle for

            // Gérer un bloc potentiellement incomplet à la fin
            if (currentBlock.Count > 0)
            {
                 outputList.Add(new DecoderOutputEvent(currentBlock.First().StartIndex, lastFrameEndIndex, DecoderOutputColor.Orange, "Incomplete Block?"));
            }
        }

        // TODO: Ajouter d'autres fonctions helper pour :
        // - Interpréter les commandes MDB spécifiques (Changer, Bill Validator, etc.)
        // - Formater les sorties pour l'affichage
    }
}
