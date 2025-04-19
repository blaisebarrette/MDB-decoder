///////////////////
//
//  UART/RS232 decoder by Robert44
//  https://github.com/robert44/decoders
//

namespace LabNation.Decoders
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel.Composition;
    using System.Diagnostics;
    using System.Linq;
    using System.Text;

    using LabNation.Interfaces;

    /// <summary>
    /// The MDB decoder.
    /// </summary>
    [Export(typeof(IProcessor))]
    public class DecoderMDB : IDecoder
    {
        /// <summary>
        /// Gets the description.
        /// </summary>
        public DecoderDescription Description
        {
            get
            {
                return new DecoderDescription
                {
                    Name = "MDB decoder",
                    ShortName = "MDB",
                    Author = "Blaise",
                    VersionMajor = 0,
                    VersionMinor = 1,
                    Description = "Multi-Drop Bus (MDB) protocol decoder for vending machine peripherals.",
                    InputWaveformTypes = new Dictionary<string, Type> { { "Input", typeof(bool) } },
                    // MDB has fixed settings (9600 baud, 8 data bits, 1 mode bit, 1 stop bit, no parity)
                    // Parameters might be added later for filtering specific device addresses, etc.
                    Parameters = new DecoderParameter[]
                    {
                       // Removed UART specific parameters like Baud, Bits, Parity, Stopbits, Mode
                    }
                };
            }
        }

        /// <summary>
        /// The decoding method.
        /// </summary>
        /// <param name="inputWaveforms"> The input waveforms. </param>
        /// <param name="parameters"> The parameters. </param>
        /// <param name="samplePeriod"> The sample period. </param>
        /// <returns> The output returned to the scope. </returns>
        public DecoderOutput[] Process(Dictionary<string, Array> inputWaveforms, Dictionary<string, object> parameters, double samplePeriod)
        {
            var decoderOutputList = new List<DecoderOutput>();
            const int MDB_BAUDRATE = 9600;
            // MDB timings (from Référence MDB.md)
            const double T_INTER_BYTE_MAX_MS = 1.0; // Max inter-byte time during send
            const double T_RESPONSE_MAX_MS = 5.0;   // Max response time for peripheral

            try
            {
                var serialData = (bool[])inputWaveforms["Input"];
                if (serialData == null || serialData.Length == 0)
                    return decoderOutputList.ToArray();

                // MDB uses Idle High, Active Low (implied from Break condition description)
                bool idleState = true; // Assume Idle High
                bool startBitValue = !idleState; // Start bit is low

                // Calculate samples per bit based on fixed MDB baud rate
                double samplesPerBitExact = 1.0 / (samplePeriod * MDB_BAUDRATE);
                if (samplesPerBitExact < 2) // Need at least 2 samples to reliably detect bit center
                {
                    // Handle error - Sample rate too low for MDB
                    // Maybe add a DecoderOutputError here?
                    return decoderOutputList.ToArray();
                }
                int samplesPerBit = (int)Math.Round(samplesPerBitExact);
                int samplePointOffset = samplesPerBit / 2; // Sample roughly in the middle of the bit

                // MDB Frame: 1 Start, 8 Data, 1 Mode, 1 Stop = 11 bits total
                int frameBitCount = 11;
                int frameSampleCount = frameBitCount * samplesPerBit;

                List<MdbByte> decodedBytes = new List<MdbByte>();
                int currentIndex = 0;

                // ---- State Machine for Byte Detection ----
                // State 0: Idle (Waiting for Start Bit - falling edge)
                // State 1: In Frame (Sampling bits)
                int state = 0;
                int frameStartIndex = 0;

                while (currentIndex < serialData.Length - 1)
                {
                    if (state == 0) // Waiting for Start Bit
                    {
                        // Look for falling edge (Idle High -> Start Low)
                        if (serialData[currentIndex] == idleState && serialData[currentIndex + 1] == startBitValue)
                        {
                            frameStartIndex = currentIndex + 1; // Start bit begins here
                            // Check if there's enough data left for a full frame
                            if (frameStartIndex + frameSampleCount <= serialData.Length)
                            {
                                state = 1; // Move to In Frame state
                                currentIndex = frameStartIndex + samplePointOffset; // Move to first data bit sample point
                            }
                            else
                            {
                                // Not enough data for a full frame, stop searching
                                break;
                            }
                        }
                        else
                        {
                            currentIndex++; // Keep searching for start bit
                        }
                    }
                    else if (state == 1) // In Frame
                    {
                        byte currentByteValue = 0;
                        bool modeBit = false;
                        bool stopBitOk = false;
                        int bitErrors = 0;

                        // --- Corrected Sample Point Calculation ---
                        // Calculate the index where the first data bit (Bit 0) *starts*
                        int startOfBit0Index = frameStartIndex + samplesPerBit;

                        // Sample 8 Data Bits (LSB first)
                        for (int bit = 0; bit < 8; bit++)
                        {
                            // Calculate the center sample point for the current data bit
                            int sampleIndex = startOfBit0Index + (bit * samplesPerBit) + samplePointOffset;
                            if (sampleIndex >= serialData.Length) goto EndProcessing; // Check bounds

                            if (serialData[sampleIndex] == idleState) // Bit is high (1)
                                currentByteValue |= (byte)(1 << bit);
                            // else bit is low (0) - already initialized
                        }

                        // Sample Mode Bit (9th bit after start bit)
                        // Center is 8.5 bit times after start of Bit 0
                        int modeBitSampleIndex = startOfBit0Index + (8 * samplesPerBit) + samplePointOffset;
                        if (modeBitSampleIndex >= serialData.Length) goto EndProcessing; // Check bounds
                        modeBit = (serialData[modeBitSampleIndex] == idleState); // High level == 1

                        // Check Stop Bit (10th bit after start bit)
                        // Center is 9.5 bit times after start of Bit 0
                        int stopBitSampleIndex = startOfBit0Index + (9 * samplesPerBit) + samplePointOffset;
                        stopBitOk = false; // Assume error until proven otherwise
                        if (stopBitSampleIndex >= 0 && stopBitSampleIndex < serialData.Length)
                        {
                             stopBitOk = (serialData[stopBitSampleIndex] == idleState); // Stop bit should be high (idle)
                        }
                        // --- End Corrected Sample Point Calculation ---

                        /* --- Original Sampling Logic (Commented Out) ---
                        for (int bit = 0; bit < 8; bit++)
                        {
                            int sampleIndex = frameStartIndex + samplePointOffset + (bit * samplesPerBit);
                            if (sampleIndex >= serialData.Length) goto EndProcessing; // Check bounds

                            if (serialData[sampleIndex] != startBitValue) // Bit is high (1)
                                currentByteValue |= (byte)(1 << bit);
                        }
                        int modeBitSampleIndex = frameStartIndex + samplePointOffset + (8 * samplesPerBit);
                        if (modeBitSampleIndex >= serialData.Length) goto EndProcessing; // Check bounds
                        modeBit = (serialData[modeBitSampleIndex] != startBitValue);
                        int stopBitSampleIndex = frameStartIndex + samplePointOffset + (10 * samplesPerBit);
                        stopBitOk = false; // Assume error until proven otherwise
                        if (stopBitSampleIndex >= 0 && stopBitSampleIndex < serialData.Length)
                        {
                             stopBitOk = (serialData[stopBitSampleIndex] == idleState);
                        }
                        */

                        if (stopBitOk)
                        {
                            decodedBytes.Add(new MdbByte(frameStartIndex, frameSampleCount, currentByteValue, modeBit));
                        }
                        else
                        {
                            // Framing error - could log this or add specific error output
                             decoderOutputList.Add(new DecoderOutputEvent(
                                frameStartIndex,
                                frameStartIndex + frameSampleCount,
                                DecoderOutputColor.Red,
                                "Framing Error"));
                        }

                        // Move currentIndex to the end of the stop bit, ready for next idle/start bit search
                        currentIndex = frameStartIndex + frameSampleCount;
                        state = 0; // Go back to Idle state
                    }
                }

                EndProcessing:;

                // ---- Block Assembly and Interpretation ----
                if (decodedBytes.Count > 0)
                {
                    List<MdbBlock> blocks = AssembleBlocks(decodedBytes, samplePeriod, T_INTER_BYTE_MAX_MS);
                    ProcessBlocks(blocks, decoderOutputList);
                }
            }
            catch (Exception e)
            {
                // Log the exception or add a DecoderOutputError
                 decoderOutputList.Add(new DecoderOutputEvent(0, 0, DecoderOutputColor.Red, "Decoder Error: " + e.Message));
                Debug.WriteLine("MDB Decoder Exception: " + e);
            }

            return decoderOutputList.ToArray();
        }

        // ---- Helper Methods for MDB ----

        /// <summary>
        /// Represents a single decoded MDB byte with its mode bit.
        /// </summary>
        private class MdbByte
        {
            public int StartIndex { get; }
            public int SampleCount { get; }
            public byte Value { get; }
            public bool ModeBit { get; }

            public int EndIndex => StartIndex + SampleCount;

            public MdbByte(int startIndex, int sampleCount, byte value, bool modeBit)
            {
                StartIndex = startIndex;
                SampleCount = sampleCount;
                Value = value;
                ModeBit = modeBit;
            }

             public override string ToString() => $"0x{Value:X2} (Mode={(ModeBit ? 1:0)}) @{StartIndex}";
        }

        /// <summary>
        /// Represents a block of MDB communication (Master->Peripheral or Peripheral->Master).
        /// </summary>
        private class MdbBlock
        {
            public List<MdbByte> Bytes { get; } = new List<MdbByte>();
            public bool IsMasterToPeripheral { get; private set; } = false;
            public int StartIndex => Bytes.FirstOrDefault()?.StartIndex ?? 0;
            public int EndIndex => Bytes.LastOrDefault()?.EndIndex ?? 0;

            public MdbBlock(MdbByte firstByte)
            {
                Bytes.Add(firstByte);
                // Tentatively determine direction based on the first byte's mode bit
                // Mode=1 for the first byte usually means Master->Peripheral (Address byte)
                // But ACK/NAK from Peripheral also have Mode=1. Refine later if needed.
                IsMasterToPeripheral = firstByte.ModeBit && firstByte.Value != 0x00 && firstByte.Value != 0xFF;
            }

            public void AddByte(MdbByte mdbByte)
            {
                Bytes.Add(mdbByte);
            }

            public override string ToString() => $"{(IsMasterToPeripheral ? "M->P":"P->M")} Block ({Bytes.Count} bytes) @{StartIndex}";

        }


        /// <summary>
        /// Assembles individual MDB bytes into communication blocks based on timing.
        /// </summary>
        private List<MdbBlock> AssembleBlocks(List<MdbByte> bytes, double samplePeriod, double maxInterByteTimeMs)
        {
            List<MdbBlock> blocks = new List<MdbBlock>();
            if (bytes.Count == 0) return blocks;

            double maxInterByteSamples = (maxInterByteTimeMs / 1000.0) / samplePeriod;

            MdbBlock currentBlock = new MdbBlock(bytes[0]);
            blocks.Add(currentBlock);

            for (int i = 1; i < bytes.Count; i++)
            {
                double samplesBetween = bytes[i].StartIndex - bytes[i-1].EndIndex;
                if (samplesBetween <= maxInterByteSamples)
                {
                    currentBlock.AddByte(bytes[i]);
                }
                else
                {
                    // Start a new block
                    currentBlock = new MdbBlock(bytes[i]);
                    blocks.Add(currentBlock);
                }
            }
            return blocks;
        }

         /// <summary>
        /// Processes assembled MDB blocks, calculates checksums, and generates decoder output.
        /// </summary>
        private void ProcessBlocks(List<MdbBlock> blocks, List<DecoderOutput> outputList)
        {
            foreach (var block in blocks)
            {
                if (block.Bytes.Count == 0) continue;

                MdbByte firstByte = block.Bytes[0];
                int blockStartIndex = block.StartIndex;
                int blockEndIndex = block.EndIndex;

                // --- Handle Single-Byte Peripheral Responses (ACK/NAK) ---
                if (block.Bytes.Count == 1 && firstByte.ModeBit)
                {
                    if (firstByte.Value == 0x00) // ACK
                    {
                        outputList.Add(new DecoderOutputEvent(blockStartIndex, blockEndIndex, DecoderOutputColor.Green, "ACK"));
                        continue; // Processed this block
                    }
                    else if (firstByte.Value == 0xFF) // NAK
                    {
                        outputList.Add(new DecoderOutputEvent(blockStartIndex, blockEndIndex, DecoderOutputColor.Red, "NAK"));
                        continue; // Processed this block
                    }
                    // If single byte, Mode=1, but not ACK/NAK, it's likely an incomplete/invalid block start
                    // Or could be JUST RESET (0x00 with Mode=1) *after* a P->M block - needs context.
                    // For simplicity now, treat as potentially invalid if direction wasn't clear
                     outputList.Add(new DecoderOutputEvent(blockStartIndex, blockEndIndex, DecoderOutputColor.Orange, $"??? (0x{firstByte.Value:X2} M=1)"));

                }

                // --- Handle Multi-Byte Blocks (Master->Peripheral or Peripheral->Master Data) ---
                if (block.Bytes.Count > 1)
                {
                    byte calculatedChk = 0;
                    MdbByte lastByte = block.Bytes.Last();
                    MdbByte potentialChkByte = lastByte; // In MDB, CHK is the last byte visually

                    // Determine actual data bytes and CHK byte
                    List<MdbByte> dataBytesForChk = new List<MdbByte>();
                    bool checksumExpected = true;

                    if (block.IsMasterToPeripheral)
                    {
                         // M->P: CHK = SUM(Address + Data Bytes)
                         // Address byte is firstByte, potentialChkByte is lastByte
                         if (firstByte.ModeBit && !potentialChkByte.ModeBit) // Valid M->P structure
                         {
                             dataBytesForChk.Add(firstByte); // Include Address byte
                             for(int i=1; i < block.Bytes.Count -1; i++) // Add data bytes
                             {
                                 dataBytesForChk.Add(block.Bytes[i]);
                             }
                         } else {
                             checksumExpected = false; // Invalid structure for checksum
                         }
                    }
                    else // Peripheral->Master
                    {
                        // P->M: CHK = SUM(Data Bytes)
                        // Mode=1 on last *data* byte, CHK follows (Mode=0)
                        checksumExpected = false; // Assume no checksum until proven otherwise
                        int lastDataByteIndex = -1;
                        for(int i=0; i < block.Bytes.Count; i++)
                        {
                            if (block.Bytes[i].ModeBit) {
                                lastDataByteIndex = i;
                                        break;
                                }
                            }

                        // Check if CHK byte exists after the last data byte
                        if (lastDataByteIndex != -1 && lastDataByteIndex < block.Bytes.Count - 1)
                        {
                            potentialChkByte = block.Bytes[lastDataByteIndex + 1];
                            if (!potentialChkByte.ModeBit) // CHK byte should have Mode=0
                            {
                                checksumExpected = true;
                                for(int i=0; i <= lastDataByteIndex; i++) // Sum data bytes up to the one with Mode=1
                                {
                                    dataBytesForChk.Add(block.Bytes[i]);
                                }
                            }
                        }

                    }

                    // Calculate Checksum if expected
                    if (checksumExpected)
                    {
                         foreach (var b in dataBytesForChk)
                         {
                             calculatedChk = (byte)(calculatedChk + b.Value); // Modulo 256 sum
                         }
                    }

                    // --- Generate Output Events for the Block ---
                    DecoderOutputColor blockColor = block.IsMasterToPeripheral ? DecoderOutputColor.Blue : DecoderOutputColor.Green;
                    // We will now output each byte individually
                    // Remove the old block-level label/tooltip builders
                    // StringBuilder blockLabel = new StringBuilder(directionPrefix);
                    // StringBuilder blockTooltip = new StringBuilder();

                    // Keep track of checksum validity
                    bool chkOk = false;
                    if (checksumExpected)
                    {
                        chkOk = (calculatedChk == potentialChkByte.Value);
                    }

                    for(int i=0; i < block.Bytes.Count; i++)
                    {
                        MdbByte currentByte = block.Bytes[i];
                        string detail = string.Empty; // Detail string for DecoderOutputValueNumeric

                        // Determine byte type for coloring and detail string
                        bool isAddr = i == 0 && block.IsMasterToPeripheral && currentByte.ModeBit;
                        // Adjust lastDataByteIndex check for robustness
                        int actualLastDataByteIndex = -1;
                        if (!block.IsMasterToPeripheral && checksumExpected) {
                             actualLastDataByteIndex = dataBytesForChk.Count - 1;
                        }
                        bool isLastData = !block.IsMasterToPeripheral && i == actualLastDataByteIndex && currentByte.ModeBit;
                        bool isChk = i == block.Bytes.Count - 1 && checksumExpected && !currentByte.ModeBit;
                        bool isData = !isAddr && !isLastData && !isChk;

                        DecoderOutputColor byteColor = blockColor;

                        if (isAddr) { detail = "Addr"; byteColor = DecoderOutputColor.DarkBlue; }
                        if (isData) { detail = "Data"; byteColor = blockColor; }
                        if (isLastData) { detail = "LastData"; byteColor = DecoderOutputColor.Green; }
                        if (isChk) { detail = "CHK"; byteColor = chkOk ? DecoderOutputColor.Black : DecoderOutputColor.Red; }

                        // Add Mode bit info to detail
                        detail += $" M={(currentByte.ModeBit ? 1 : 0)}";

                        // Add individual byte value output
                        outputList.Add(new DecoderOutputValueNumeric(
                            currentByte.StartIndex,
                            currentByte.EndIndex,
                            byteColor,
                            currentByte.Value,
                            detail,
                            8 // Specify 8 data bits for formatting
                            ));
                    }

                    // Add a separate event for checksum status if checksum was expected
                    if (checksumExpected)
                    {
                        outputList.Add(new DecoderOutputEvent(
                            potentialChkByte.StartIndex, // Span the checksum byte
                            potentialChkByte.EndIndex,
                            chkOk ? DecoderOutputColor.Black : DecoderOutputColor.Red,
                            chkOk ? "CHK OK" : $"CHK ERR (exp {calculatedChk:X2})"));
                    }
                    else if (block.Bytes.Count > 1) // Only mark if it wasn't ACK/NAK
                    {
                         // Indicate if checksum wasn't expected or block structure seemed wrong
                         outputList.Add(new DecoderOutputEvent(
                             block.EndIndex - (block.Bytes.LastOrDefault()?.SampleCount ?? 0), // Span the last byte
                             block.EndIndex,
                             DecoderOutputColor.Orange,
                             "No CHK?"));
                    }

                    // Remove the old overall block representation
                    // string detail = $"{blockLabel.ToString().Trim()}\n{blockTooltip.ToString()}";
                    // outputList.Add(new DecoderOutputValueNumeric(blockStartIndex, blockEndIndex, blockColor, 0, detail, 0));

                } else if (block.Bytes.Count == 1 && !firstByte.ModeBit) {
                     // Single byte with Mode=0 - likely invalid/noise
                     outputList.Add(new DecoderOutputEvent(blockStartIndex, blockEndIndex, DecoderOutputColor.Orange, $"??? (0x{firstByte.Value:X2} M=0)"));
                }


            }
        }

        /// <summary>
        /// The parity (Not used in MDB, kept for reference if needed elsewhere).
        /// </summary>
        private enum Parity
        {
            None,
            Odd,
            Even,
            Mark,
            Space
        }
        
        /// <summary>
        /// The bit (Not directly used in MDB logic, can be removed or kept for debugging).
        /// </summary>
        private class Bit
        {
            /// <summary>
            /// Initializes a new instance of the <see cref="Bit"/> class.
            /// </summary>
            /// <param name="index"> The index. </param>
            /// <param name="length"> The length. </param>
            /// <param name="val"> The val. </param>
            public Bit(int index, int length, bool val)
            {
                this.Index = index;
                this.Value = val;
                this.Length = length;
            }

            /// <summary>
            /// Gets the index.
            /// </summary>
            public int Index { get; private set; }

            /// <summary>
            /// Gets the value.
            /// </summary>
            public bool Value { get; private set; }

            /// <summary>
            /// Gets the length.
            /// </summary>
            public int Length { get; private set; }

            /// <summary>
            /// The to string.
            /// </summary>
            /// <returns>
            /// The <see cref="string"/>.
            /// </returns>
            public override string ToString()
            {
                return string.Format("{0},{1},{2}", this.Index, this.Length, this.Value);
            }
        }
    }
}