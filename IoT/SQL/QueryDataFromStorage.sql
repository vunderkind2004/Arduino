/****** Script for SelectTopNRows command from SSMS  ******/
SELECT TOP (1000) 
	   CONVERT(datetime, SWITCHOFFSET(IoTUtcTime, DATEPART(TZOFFSET, 
			IoTUtcTime AT TIME ZONE 'E. Europe Standard Time')))
       AS IoT_LocalTime
	   ,[DeviceId]
	  ,[temperature]
      ,[humidity]
      --,[EventProcessedUtcTime]
      --,[PartitionId]
      --,[EventEnqueuedUtcTime]
      --,[IoTHub]      
      --,[IoTUtcTime] 
  FROM [dbo].[testData]
  where deviceid= 'Realdevice1'
  order by [EventProcessedUtcTime] desc

  --select * from sys.time_zone_info