import { useState, useRef, useCallback } from 'react';

export interface UwbData {
  time: number;
  cyc: number;
  ops: number;
  a1Raw: number;
  a1Filter: number;
  a2Raw: number;
  a2Filter: number;
  a3Raw: number;
  a3Filter: number;
  a4Raw: number;
  a4Filter: number;
}

export function useWebSerial(maxHistory: number = 120) {
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [dataHistory, setDataHistory] = useState<UwbData[]>([]);
  const [latestData, setLatestData] = useState<UwbData | null>(null);
  const [isLogging, setIsLogging] = useState(false);

  const portRef = useRef<any>(null);
  const readerRef = useRef<any>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const keepReadingRef = useRef(false);
  const csvLogRef = useRef<UwbData[]>([]);
  const isLoggingRef = useRef(false);

  // Regex based on the implementation plan
  // Cyc: 50 | Ops: 190 | A1: 2223(2223) | A2: 1958(1890) | A3: 1590(1606) | A4: 2450(2438)
  const regex = /Cyc:\s*(\d+)\s*\|\s*Ops:\s*(\d+)\s*\|\s*A1:\s*(\d+)\((\d+)\)\s*\|\s*A2:\s*(\d+)\((\d+)\)\s*\|\s*A3:\s*(\d+)\((\d+)\)\s*\|\s*A4:\s*(\d+)\((\d+)\)/;

  const parseLine = useCallback((line: string) => {
    const match = line.match(regex);
    if (match) {
      const newData: UwbData = {
        time: Date.now(),
        cyc: parseInt(match[1], 10),
        ops: parseInt(match[2], 10),
        a1Raw: parseInt(match[3], 10),
        a1Filter: parseInt(match[4], 10),
        a2Raw: parseInt(match[5], 10),
        a2Filter: parseInt(match[6], 10),
        a3Raw: parseInt(match[7], 10),
        a3Filter: parseInt(match[8], 10),
        a4Raw: parseInt(match[9], 10),
        a4Filter: parseInt(match[10], 10),
      };

      setLatestData(newData);
      setDataHistory(prev => {
        const newHistory = [...prev, newData];
        if (newHistory.length > maxHistory) {
          return newHistory.slice(newHistory.length - maxHistory);
        }
        return newHistory;
      });

      // If logging, push to the CSV log buffer (no size limit)
      if (isLoggingRef.current) {
        csvLogRef.current.push(newData);
      }
    }
  }, [maxHistory, regex]);

  const connect = async () => {
    try {
      if (!('serial' in navigator)) {
        throw new Error('Web Serial API is not supported in this browser. Please use Chrome or Edge.');
      }

      const port = await (navigator as any).serial.requestPort();
      await port.open({ baudRate: 115200 });
      
      portRef.current = port;
      keepReadingRef.current = true;
      setIsConnected(true);
      setError(null);
      
      readLoop();
    } catch (err: any) {
      console.error(err);
      setError(err.message || 'Failed to connect via USB');
    }
  };

  const connectWifi = (ipAddress: string) => {
    try {
      const wsUrl = `ws://${ipAddress}:81`;
      const ws = new WebSocket(wsUrl);
      
      ws.onopen = () => {
        setIsConnected(true);
        setError(null);
      };
      
      ws.onmessage = (event) => {
        const data = event.data;
        if (typeof data === 'string') {
          const lines = data.split('\n');
          for (const line of lines) {
            if (line.trim()) {
              parseLine(line.trim());
            }
          }
        }
      };
      
      ws.onerror = () => {
        setError('WebSocket error. Is ESP32 running?');
        setIsConnected(false);
      };
      
      ws.onclose = () => {
        setIsConnected(false);
      };
      
      wsRef.current = ws;
    } catch (err: any) {
      console.error(err);
      setError(err.message || 'Failed to connect via WiFi');
    }
  };

  const disconnect = async () => {
    keepReadingRef.current = false;
    if (readerRef.current) {
      await readerRef.current.cancel();
    }
    if (portRef.current) {
      await portRef.current.close();
    }
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    setIsConnected(false);
    portRef.current = null;
  };

  const readLoop = async () => {
    const port = portRef.current;
    if (!port) return;

    const textDecoder = new TextDecoderStream();
    port.readable.pipeTo(textDecoder.writable);
    const reader = textDecoder.readable.getReader();
    readerRef.current = reader;

    let buffer = '';

    try {
      while (keepReadingRef.current) {
        const { value, done } = await reader.read();
        if (done) {
          break;
        }
        
        buffer += value;
        const lines = buffer.split('\n');
        
        // Keep the last incomplete line in the buffer
        buffer = lines.pop() || '';
        
        for (const line of lines) {
          parseLine(line.trim());
        }
      }
    } catch (error) {
      console.error('Error reading from serial port', error);
      setError('Connection lost');
      setIsConnected(false);
    } finally {
      reader.releaseLock();
    }
  };

  const startLog = () => {
    csvLogRef.current = [];
    isLoggingRef.current = true;
    setIsLogging(true);
  };

  const stopLog = () => {
    isLoggingRef.current = false;
    setIsLogging(false);
  };

  const exportCsv = () => {
    const log = csvLogRef.current;
    if (log.length === 0) {
      alert('No data to export. Start logging first.');
      return;
    }

    const header = 'Timestamp,Time_ISO,Cyc_Hz,Ops_per_s,A1_Raw_mm,A1_Filter_mm,A2_Raw_mm,A2_Filter_mm,A3_Raw_mm,A3_Filter_mm,A4_Raw_mm,A4_Filter_mm';
    const rows = log.map(d => [
      d.time,
      new Date(d.time).toISOString(),
      d.cyc,
      d.ops,
      d.a1Raw,
      d.a1Filter,
      d.a2Raw,
      d.a2Filter,
      d.a3Raw,
      d.a3Filter,
      d.a4Raw,
      d.a4Filter,
    ].join(','));

    const csvContent = [header, ...rows].join('\n');
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    const filename = `uwb_log_${new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)}.csv`;
    link.setAttribute('href', url);
    link.setAttribute('download', filename);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  return {
    isConnected,
    error,
    latestData,
    dataHistory,
    connect,
    connectWifi,
    disconnect,
    isLogging,
    startLog,
    stopLog,
    exportCsv,
  };
}
